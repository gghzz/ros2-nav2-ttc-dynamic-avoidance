#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo
{
class MovingObstaclePlugin : public ModelPlugin
{
public:
  MovingObstaclePlugin() = default;
  ~MovingObstaclePlugin() override = default;

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    if (!_model)
    {
      gzerr << "[MovingObstaclePlugin] Model pointer is null.\n";
      return;
    }

    this->model_ = _model;

    // 读取运动轴：x 或 y，默认 x
    if (_sdf->HasElement("motion_axis"))
      this->motion_axis_ = _sdf->Get<std::string>("motion_axis");
    else
      this->motion_axis_ = "x";

    // 为了兼容你之前那版 x 方向参数，x 方向仍支持 start_x/end_x + y
    this->start_x_ = _sdf->HasElement("start_x") ? _sdf->Get<double>("start_x") : 0.0;
    this->end_x_   = _sdf->HasElement("end_x")   ? _sdf->Get<double>("end_x")   : 2.0;
    this->start_y_ = _sdf->HasElement("start_y") ? _sdf->Get<double>("start_y") : 0.0;
    this->end_y_   = _sdf->HasElement("end_y")   ? _sdf->Get<double>("end_y")   : 2.0;

    // 固定坐标
    this->fixed_y_ = _sdf->HasElement("y") ? _sdf->Get<double>("y") : 0.0;
    this->fixed_x_ = _sdf->HasElement("x") ? _sdf->Get<double>("x") : 0.0;

    this->z_ = _sdf->HasElement("z") ? _sdf->Get<double>("z") : 0.5;
    this->yaw_ = _sdf->HasElement("yaw") ? _sdf->Get<double>("yaw") : 0.0;
    this->speed_ = _sdf->HasElement("speed") ? _sdf->Get<double>("speed") : 0.3;

    // 初始方向：正向
    this->forward_ = true;

    // 初始化模型位置
    ignition::math::Pose3d initial_pose = this->model_->WorldPose();

    if (this->motion_axis_ == "y" || this->motion_axis_ == "Y")
    {
      initial_pose.Pos().X(this->fixed_x_);
      initial_pose.Pos().Y(this->start_y_);
    }
    else
    {
      // 默认 x 方向
      initial_pose.Pos().X(this->start_x_);
      initial_pose.Pos().Y(this->fixed_y_);
    }

    initial_pose.Pos().Z(this->z_);
    initial_pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, this->yaw_);
    this->model_->SetWorldPose(initial_pose);

    this->last_update_time_ = this->model_->GetWorld()->SimTime();

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&MovingObstaclePlugin::OnUpdate, this));

    gzmsg << "[MovingObstaclePlugin] Loaded for model ["
          << this->model_->GetName()
          << "], axis=" << this->motion_axis_
          << ", speed=" << this->speed_ << "\n";
  }

private:
  void OnUpdate()
  {
    if (!this->model_)
      return;

    common::Time current_time = this->model_->GetWorld()->SimTime();
    double dt = (current_time - this->last_update_time_).Double();
    this->last_update_time_ = current_time;

    if (dt <= 0.0)
      return;

    ignition::math::Pose3d pose = this->model_->WorldPose();

    if (this->motion_axis_ == "y" || this->motion_axis_ == "Y")
    {
      double y = pose.Pos().Y();

      if (this->forward_)
      {
        y += this->speed_ * dt;
        if (y >= this->end_y_)
        {
          y = this->end_y_;
          this->forward_ = false;
        }
      }
      else
      {
        y -= this->speed_ * dt;
        if (y <= this->start_y_)
        {
          y = this->start_y_;
          this->forward_ = true;
        }
      }

      pose.Pos().X(this->fixed_x_);
      pose.Pos().Y(y);
    }
    else
    {
      // 默认 x 方向
      double x = pose.Pos().X();

      if (this->forward_)
      {
        x += this->speed_ * dt;
        if (x >= this->end_x_)
        {
          x = this->end_x_;
          this->forward_ = false;
        }
      }
      else
      {
        x -= this->speed_ * dt;
        if (x <= this->start_x_)
        {
          x = this->start_x_;
          this->forward_ = true;
        }
      }

      pose.Pos().X(x);
      pose.Pos().Y(this->fixed_y_);
    }

    pose.Pos().Z(this->z_);
    pose.Rot() = ignition::math::Quaterniond(0.0, 0.0, this->yaw_);

    this->model_->SetWorldPose(pose);
  }

private:
  physics::ModelPtr model_;
  event::ConnectionPtr update_connection_;
  common::Time last_update_time_;

  std::string motion_axis_{"x"};

  double start_x_{0.0};
  double end_x_{2.0};
  double start_y_{0.0};
  double end_y_{2.0};

  double fixed_x_{0.0};
  double fixed_y_{0.0};

  double z_{0.5};
  double yaw_{0.0};
  double speed_{0.3};

  bool forward_{true};
};

GZ_REGISTER_MODEL_PLUGIN(MovingObstaclePlugin)
}