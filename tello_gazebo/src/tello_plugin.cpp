#include <chrono>
#include <map>
#include <string>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <cmath>

#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo_ros/node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tello_msgs/msg/flight_data.hpp"
#include "tello_msgs/msg/tello_response.hpp"
#include "tello_msgs/srv/tello_action.hpp"

#include "pid.hpp"

using namespace std::chrono_literals;

//===============================================================================================
// TelloPlugin features:
// -- generates trivial flight data at 10Hz
// -- video is managed by a gazebo_ros_pkgs plugin, see tello_description/urdf/tello.xml
// -- responds to "takeoff" and "land" commands
// -- responds to cmd_vel and "rc x y z yaw" commands
// -- responds to "go dx dy dz dyaw" commands
// -- battery state
//
// Tello flight dynamics are sophisticated and difficult to model. TelloPlugin keeps it simple:
// -- x, y, z and yaw velocities are controlled by a P controller
// -- roll and pitch are always 0
//
// Possible extensions:
// -- simulate network latency
// -- add some randomness to the flight dynamics
// -- improve battery simulation
//===============================================================================================

namespace tello_gazebo
{

  //=============================================================================================
  // Configurações gerais
  //=============================================================================================
  const double MAX_XY_V   = 8.0;
  const double MAX_Z_V    = 4.0;
  const double MAX_ANG_V  = M_PI;

  const double MAX_XY_A   = 8.0;
  const double MAX_Z_A    = 4.0;
  const double MAX_ANG_A  = M_PI;

  const double TAKEOFF_Z   = 1.0;     // z alvo no decolagem
  const double TAKEOFF_Z_V = 0.5;

  const double LAND_Z      = 0.1;     // z alvo no pouso
  const double LAND_Z_V    = -0.5;

  const int    BATTERY_DURATION = 6000;  // segundos

  // Go-to
  const double GO_POS_TOL  = 0.05;   // tolerância de posição (m)
  const double GO_KP_SPEED = 1.0;    // ganho p/ reduzir vel perto do alvo (m/s por m)

  inline double clamp(const double v, const double max)
  {
    return v > max ? max : (v < -max ? -max : v);
  }

  inline double clamp_minmax(const double v, const double minv, const double maxv)
  {
    return std::max(minv, std::min(maxv, v));
  }

  class TelloPlugin : public gazebo::ModelPlugin
  {
    enum class FlightState
    {
      landed,
      taking_off,
      flying,
      landing,
      dead_battery,
    };

    std::map<FlightState, const char *> state_strs_{
      {FlightState::landed,       "landed"},
      {FlightState::taking_off,   "taking_off"},
      {FlightState::flying,       "flying"},
      {FlightState::landing,      "landing"},
      {FlightState::dead_battery, "dead_battery"},
    };

    FlightState flight_state_;

    gazebo::physics::LinkPtr base_link_;
    ignition::math::Vector3d gravity_;
    ignition::math::Vector3d center_of_mass_{0, 0, 0};
    int battery_duration_{BATTERY_DURATION};

    // Conexão com o update do Gazebo
    gazebo::event::ConnectionPtr update_connection_;

    // Nó ROS
    gazebo_ros::Node::SharedPtr node_;

    // Publicadores ROS
    rclcpp::Publisher<tello_msgs::msg::FlightData>::SharedPtr flight_data_pub_;
    rclcpp::Publisher<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_pub_;

    // Serviço ROS
    rclcpp::Service<tello_msgs::srv::TelloAction>::SharedPtr command_srv_;

    // Assinaturas ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Tempos de simulação
    gazebo::common::Time update_time_;

    // 10Hz timer
    gazebo::common::Time ten_hz_time_;

    // Controladores de velocidade (frame do corpo)
    pid::Controller x_controller_{false, 2, 0, 0};
    pid::Controller y_controller_{false, 2, 0, 0};
    pid::Controller z_controller_{false, 2, 0, 0};
    pid::Controller yaw_controller_{false, 2, 0, 0};

    // -------------------
    // Estado do comando GO
    // -------------------
    bool   go_active_{false};
    ignition::math::Vector3d go_target_world_{0,0,0};  // alvo em coordenadas do mundo
    double go_speed_limit_{0.5};                       // m/s (entrada é cm/s, convertida)
    double go_pos_tol_{GO_POS_TOL};

    // -------------------
    // Debug logging
    // -------------------
    bool debug_{false};
    double debug_go_rate_hz_{2.0};
    gazebo::common::Time debug_log_time_;

  public:

    TelloPlugin()
    {
      (void) update_connection_;
      (void) command_srv_;
      (void) cmd_vel_sub_;

      transition(FlightState::landed);
    }

    ~TelloPlugin() = default;

    // Helper de log condicionado
    template<typename... Args>
    void DBG(const char* fmt, Args... args)
    {
      if (debug_ && node_ != nullptr) {
        RCLCPP_INFO(node_->get_logger(), fmt, args...);
      }
    }

    void set_target_velocities(double x, double y, double z, double yaw)
    {
      x_controller_.set_target(x);
      y_controller_.set_target(y);
      z_controller_.set_target(z);
      yaw_controller_.set_target(yaw);
    }

    // Responde ao comando "rc x y z yaw"
    void set_target_velocities(std::string rc_command)
    {
      // cancelar GO se houver um RC manual
      if (go_active_) {
        DBG("GO canceled by RC command");
      }
      go_active_ = false;

      double x, y, z, yaw;

      try {
        std::istringstream iss(rc_command, std::istringstream::in);
        std::string s;
        iss >> s; // "rc"
        iss >> s; x   = std::stof(s);
        iss >> s; y   = std::stof(s);
        iss >> s; z   = std::stof(s);
        iss >> s; yaw = std::stof(s);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "can't parse rc command '%s', exception %s", rc_command.c_str(), e.what());
        return;
      }

      set_target_velocities(
        x * MAX_XY_V,
        y * MAX_XY_V,
        z * MAX_Z_V,
        yaw * MAX_ANG_V);
    }

    void transition(FlightState next)
    {
      if (node_ != nullptr) {
        RCLCPP_INFO(node_->get_logger(), "transition from '%s' to '%s'", state_strs_[flight_state_], state_strs_[next]);
      }

      flight_state_ = next;

      switch (flight_state_) {
        case FlightState::landed:
        case FlightState::flying:
        case FlightState::dead_battery:
          set_target_velocities(0, 0, 0, 0);
          break;

        case FlightState::taking_off:
          set_target_velocities(0, 0, TAKEOFF_Z_V, 0);
          break;

        case FlightState::landing:
          set_target_velocities(0, 0, LAND_Z_V, 0);
          break;
      }

      // qualquer transição cancela um GO em andamento
      if (flight_state_ != FlightState::flying) {
        if (go_active_) {
          DBG("GO canceled by state transition");
        }
        go_active_ = false;
      }
    }

    // Chamado uma vez ao carregar o plugin
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      GZ_ASSERT(model != nullptr, "Model is null");
      GZ_ASSERT(sdf != nullptr, "SDF is null");

      std::string link_name{"base_link"};

      // In theory we can move much of this config into the <ros> tag, perhaps it's finished in Eloquent?
      if (sdf->HasElement("link_name")) {
        link_name = sdf->GetElement("link_name")->Get<std::string>();
      }
      if (sdf->HasElement("center_of_mass")) {
        center_of_mass_ = sdf->GetElement("center_of_mass")->Get<ignition::math::Vector3d>();
      }
      if (sdf->HasElement("battery_duration")) {
        // BUGFIX: antes lia de "center_of_mass"
        battery_duration_ = sdf->GetElement("battery_duration")->Get<int>();
      }

      base_link_ = model->GetLink(link_name);
      GZ_ASSERT(base_link_ != nullptr, "Missing link");
      gravity_ = model->GetWorld()->Gravity();

      std::cout << std::fixed << std::setprecision(2);
      std::cout << "\nTELLO PLUGIN\n";
      std::cout << "-----------------------------------------\n";
      std::cout << "link_name: " << link_name << "\n";
      std::cout << "center_of_mass: " << center_of_mass_ << "\n";
      std::cout << "gravity: " << gravity_ << "\n";
      std::cout << "battery_duration: " << battery_duration_ << "\n";
      std::cout << "-----------------------------------------\n\n";

      // Nó ROS
      node_ = gazebo_ros::Node::Get(sdf);

      // parâmetros ROS (via <ros><parameter .../> no SDF ou dynamic)
      node_->declare_parameter<bool>("debug", debug_);
      node_->declare_parameter<double>("debug_go_rate_hz", debug_go_rate_hz_);
      node_->get_parameter("debug", debug_);
      node_->get_parameter("debug_go_rate_hz", debug_go_rate_hz_);
      if (debug_go_rate_hz_ <= 0.0) debug_go_rate_hz_ = 2.0;

      // use_sim_time
      bool use_sim_time = false;
      node_->get_parameter("use_sim_time", use_sim_time);
      if (!use_sim_time) {
        RCLCPP_ERROR(node_->get_logger(), "use_sim_time is false, could be a bug");
      }

      // Publicadores
      flight_data_pub_    = node_->create_publisher<tello_msgs::msg::FlightData>("flight_data", 1);
      tello_response_pub_ = node_->create_publisher<tello_msgs::msg::TelloResponse>("tello_response", 1);

      // Serviço
      command_srv_ = node_->create_service<tello_msgs::srv::TelloAction>(
        "tello_action",
        std::bind(&TelloPlugin::command_callback, this,
                  std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

      // Assinaturas
      cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&TelloPlugin::cmd_vel_callback, this, std::placeholders::_1));

      // Conectar no update do mundo
      update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        boost::bind(&TelloPlugin::OnUpdate, this, _1));
    }

    // Chamado no início de cada iteração do simulador (até 1000 Hz)
    void OnUpdate(const gazebo::common::UpdateInfo &info)
    {
      // Do nothing if the battery is dead
      if (flight_state_ == FlightState::dead_battery) {
        return;
      }

      // Timer 10Hz
      if ((info.simTime - ten_hz_time_).Double() > 0.1) {
        spin_10Hz();
        ten_hz_time_ = info.simTime;
      }

      // dt
      double dt = (info.simTime - update_time_).Double();
      update_time_ = info.simTime;

      // Não aplica força se está pousado
      if (flight_state_ != FlightState::landed) {
        // Velocidade atual no frame do corpo
        ignition::math::Vector3d linear_velocity = base_link_->RelativeLinearVel();
        ignition::math::Vector3d angular_velocity = base_link_->RelativeAngularVel();

        // Se há um GO ativo, atualiza os alvos de velocidade com base no erro de posição
        if (go_active_) {
          update_go_velocity_targets();

          // Logs periódicos de debug de pose/erro/goal
          double period = 1.0 / debug_go_rate_hz_;
          if (debug_ && (info.simTime - debug_log_time_).Double() >= period) {
            ignition::math::Pose3d pose = base_link_->WorldPose();
            ignition::math::Vector3d err = go_target_world_ - pose.Pos();
            RCLCPP_INFO(node_->get_logger(),
                        "GO: CUR=(%.3f,%.3f,%.3f) GOAL=(%.3f,%.3f,%.3f) ERR=(%.3f,%.3f,%.3f)",
                        pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
                        go_target_world_.X(), go_target_world_.Y(), go_target_world_.Z(),
                        err.X(), err.Y(), err.Z());
            debug_log_time_ = info.simTime;
          }
        }

        // Calcula aceleração desejada (ubar) pelos PIDs de velocidade
        ignition::math::Vector3d lin_ubar, ang_ubar;
        lin_ubar.X(x_controller_.calc(linear_velocity.X(), dt, 0));
        lin_ubar.Y(y_controller_.calc(linear_velocity.Y(), dt, 0));
        lin_ubar.Z(z_controller_.calc(linear_velocity.Z(), dt, 0));
        ang_ubar.Z(yaw_controller_.calc(angular_velocity.Z(), dt, 0));

        // Satura aceleração
        lin_ubar.X() = clamp(lin_ubar.X(), MAX_XY_A);
        lin_ubar.Y() = clamp(lin_ubar.Y(), MAX_XY_A);
        lin_ubar.Z() = clamp(lin_ubar.Z(), MAX_Z_A);
        ang_ubar.Z() = clamp(ang_ubar.Z(), MAX_ANG_A);

        // Compensa gravidade
        lin_ubar -= gravity_;

        // Força e torque
        ignition::math::Vector3d force  = lin_ubar * base_link_->GetInertial()->Mass();
        ignition::math::Vector3d torque = ang_ubar * base_link_->GetInertial()->MOI();

        // Mantém roll e pitch = 0
        ignition::math::Pose3d pose = base_link_->WorldPose();
        pose.Rot().X(0);
        pose.Rot().Y(0);
        base_link_->SetWorldPose(pose);

        // Aplica força/torque
        base_link_->AddLinkForce(force, center_of_mass_);
        base_link_->AddRelativeTorque(torque);
      }
    }

    bool is_prefix(const std::string &prefix, const std::string &str)
    {
      if (str.size() < prefix.size()) return false;
      return std::equal(prefix.begin(), prefix.end(), str.begin());
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "UnusedValue"

    void command_callback(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<tello_msgs::srv::TelloAction::Request> request,
      std::shared_ptr<tello_msgs::srv::TelloAction::Response> response)
    {
      const std::string &cmd = request->cmd;
      RCLCPP_INFO(node_->get_logger(), "cmd=\"%s\"", cmd.c_str());

      if (cmd == "takeoff" && flight_state_ == FlightState::landed) {
        transition(FlightState::taking_off);
        response->rc = response->OK;
        return;
      }

      if (cmd == "land" && flight_state_ == FlightState::flying) {
        transition(FlightState::landing);
        response->rc = response->OK;
        return;
      }

      // Comando RC manual
      if (is_prefix("rc", cmd) && flight_state_ == FlightState::flying) {
        set_target_velocities(cmd);   // cancela GO internamente
        response->rc = response->OK;
        return;
      }

      // -------------------------------
      // Novo comando: "go dx dy dz speed [to]"   (dx/dy/dz em cm, speed em cm/s)
      // -------------------------------
      if (is_prefix("go", cmd) && flight_state_ == FlightState::flying) {
        double dx_cm = 0, dy_cm = 0, dz_cm = 0, spd_cm_s = 0;

        try {
          std::istringstream iss(cmd);
          std::string token;
          iss >> token; // "go"

          std::vector<std::string> toks;
          while (iss >> token) {
            toks.push_back(token);
          }
          // remover "to" se for o último token
          if (!toks.empty()) {
            std::string last = toks.back();
            std::transform(last.begin(), last.end(), last.begin(), ::tolower);
            if (last == "to") {
              toks.pop_back();
            }
          }

          if (toks.size() != 4) {
            RCLCPP_WARN(node_->get_logger(), "uso: go dx dy dz speed (cm/cm/s)");
            response->rc = response->ERROR_BUSY;
            return;
          }

          dx_cm   = std::stod(toks[0]);
          dy_cm   = std::stod(toks[1]);
          dz_cm   = std::stod(toks[2]);
          spd_cm_s= std::stod(toks[3]);
        } catch (const std::exception &e) {
          RCLCPP_WARN(node_->get_logger(), "ignoring go (parse error): '%s' (%s)", cmd.c_str(), e.what());
          response->rc = response->ERROR_BUSY;
          return;
        }

        // Conversão cm -> m
        const double dx = dx_cm / 100.0;
        const double dy = dy_cm / 100.0;
        const double dz = dz_cm / 100.0;
        double speed_mps = spd_cm_s / 100.0;
        speed_mps = clamp_minmax(speed_mps, 0.05, 3.0); // evita 0 e limita razoavelmente

        // Define alvo em coordenadas do mundo com base na pose atual
        ignition::math::Pose3d pose = base_link_->WorldPose();
        const double yaw = pose.Rot().Yaw();

        // deslocamento fornecido no frame do corpo (x: frente, y: esquerda, z: cima)
        ignition::math::Vector3d d_body(dx, dy, dz);
        ignition::math::Vector3d d_world(
          std::cos(yaw) * d_body.X() - std::sin(yaw) * d_body.Y(),
          std::sin(yaw) * d_body.X() + std::cos(yaw) * d_body.Y(),
          d_body.Z()
        );

        go_target_world_  = pose.Pos() + d_world;
        go_speed_limit_   = speed_mps;
        go_pos_tol_       = GO_POS_TOL;
        go_active_        = true;

        // zera yaw target (não queremos girar durante o go)
        yaw_controller_.set_target(0.0);

        // Logs estilo Python
        ignition::math::Vector3d err0 = go_target_world_ - pose.Pos();
        RCLCPP_INFO(node_->get_logger(),
                    "GO (cm): dx=%.1f dy=%.1f dz=%.1f v=%.1f -> CUR=(%.3f,%.3f,%.3f) GOAL=(%.3f,%.3f,%.3f) ERR0=(%.3f,%.3f,%.3f)",
                    dx_cm, dy_cm, dz_cm, spd_cm_s,
                    pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(),
                    go_target_world_.X(), go_target_world_.Y(), go_target_world_.Z(),
                    err0.X(), err0.Y(), err0.Z());

        response->rc = response->OK;   // responde OK ao serviço
        return;
      }

      // Comando desconhecido/indisponível
      RCLCPP_WARN(node_->get_logger(), "ignoring command '%s'", cmd.c_str());
      response->rc = response->ERROR_BUSY;
    }
#pragma clang diagnostic pop

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      if (flight_state_ == FlightState::flying) {
        // cancelar GO ao receber comando manual
        if (go_active_) {
          DBG("GO canceled by cmd_vel");
        }
        go_active_ = false;

        // TODO: cmd_vel deveria especificar velocidade e não posição de joystick
        set_target_velocities(
          msg->linear.x  * MAX_XY_V,
          msg->linear.y  * MAX_XY_V,
          msg->linear.z  * MAX_Z_V,
          msg->angular.z * MAX_ANG_V);
      }
    }

    void respond_ok()
    {
      tello_msgs::msg::TelloResponse msg;
      msg.rc = msg.OK;
      msg.str = "ok";
      tello_response_pub_->publish(msg);
    }

    // Atualiza as metas de velocidade (no frame do corpo) para seguir rumo ao alvo do GO
    void update_go_velocity_targets()
    {
      // pose e erro no mundo
      ignition::math::Pose3d pose = base_link_->WorldPose();
      ignition::math::Vector3d e_world = go_target_world_ - pose.Pos();

      // Chegou?
      if (e_world.Length() <= go_pos_tol_) {
        go_active_ = false;
        set_target_velocities(0, 0, 0, 0);
        RCLCPP_INFO(node_->get_logger(), "GO reached target (<= %.3f m).", go_pos_tol_);
        respond_ok(); // notifica conclusão (consistente com takeoff/land)
        return;
      }

      // velocidade desejada no mundo (proporcional à distância, limitada por go_speed_limit_)
      double desired_speed = std::min(go_speed_limit_, GO_KP_SPEED * e_world.Length());
      ignition::math::Vector3d v_world = e_world;
      v_world.Normalize();
      v_world *= desired_speed;

      // converter para frame do corpo (apenas yaw)
      const double yaw = pose.Rot().Yaw();
      ignition::math::Vector3d v_body(
        std::cos(yaw) * v_world.X() + std::sin(yaw) * v_world.Y(),
       -std::sin(yaw) * v_world.X() + std::cos(yaw) * v_world.Y(),
        v_world.Z()
      );

      // limitar por máximos globais do plugin
      v_body.X() = clamp(v_body.X(), MAX_XY_V);
      v_body.Y() = clamp(v_body.Y(), MAX_XY_V);
      v_body.Z() = clamp(v_body.Z(), MAX_Z_V);

      // seta metas de velocidade para os PIDs
      set_target_velocities(v_body.X(), v_body.Y(), v_body.Z(), 0.0);
    }

    void spin_10Hz()
    {
      rclcpp::Time ros_time = node_->now();

      // Espera tempo ROS ficar razoável
      if (ros_time.seconds() < 1.0) {
        return;
      }

      // Simula bateria
      int battery_percent = static_cast<int>((battery_duration_ - ros_time.seconds()) / battery_duration_ * 100);
      if (battery_percent <= 0) {
        // We're dead
        transition(FlightState::dead_battery);
        return;
      }

      // Publica dados de voo
      tello_msgs::msg::FlightData flight_data;
      flight_data.header.stamp = ros_time;
      flight_data.sdk = flight_data.SDK_1_3;
      flight_data.bat = battery_percent;
      flight_data_pub_->publish(flight_data);

      // Finaliza ações pendentes takeoff/land
      ignition::math::Pose3d pose = base_link_->WorldPose();
      if (flight_state_ == FlightState::taking_off && pose.Pos().Z() > TAKEOFF_Z) {
        transition(FlightState::flying);
        respond_ok();
      } else if (flight_state_ == FlightState::landing && pose.Pos().Z() < LAND_Z) {
        transition(FlightState::landed);
        respond_ok();
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(TelloPlugin)

} // namespace tello_gazebo
