#include "TargetManager/TargetManager.hpp"

namespace rm
{

TrackingBase::TrackingBase(int id, TargetType type, double x, double y, std::int64_t timestamp_ns)
    : m_id(id),
      m_type(type),
      m_status(TargetStatus::TRACK),
      m_x(x),
      m_y(y),
      m_predicted_x(x),
      m_predicted_y(y),
      m_last_observed_timestamp_ns(timestamp_ns)
{
}

TrackingBase::~TrackingBase() = default;

void TrackingBase::modify_type(TargetType type)
{
    m_type = type;
}

void TrackingBase::modify_status(TargetStatus status)
{
    m_status = status;
}

void TrackingBase::modify_missed_count(int count)
{
    m_missed_count = count;
}

void TrackingBase::modify_x(double x)
{
    m_x = x;
}

void TrackingBase::modify_y(double y)
{
    m_y = y;
}

void TrackingBase::modity_velocity_x(double velocity_x)
{
    m_velocity_x = velocity_x;
}

void TrackingBase::modity_velocity_y(double velocity_y)
{
    m_velocity_y = velocity_y;
}

ArmorTarget::ArmorTarget(const Observation& observation, std::int64_t timestamp_ns)
    : TrackingBase(observation.id, TargetType::ARMOR, observation.x, observation.y, timestamp_ns)
{
}

int ArmorTarget::lostThreshold() const noexcept
{
    return lostArmor;
}

SpinningTarget::SpinningTarget(const Observation& observation, std::int64_t timestamp_ns)
    : TrackingBase(observation.id, TargetType::SPINNING, observation.x, observation.y, timestamp_ns),
      m_angle(observation.angle),
      m_predicted_angle(observation.angle),
      m_last_observed_angle(observation.angle)
{
}

int SpinningTarget::lostThreshold() const noexcept
{
    return lostSpinning;
}

double SpinningTarget::get_angle() const noexcept
{
    return m_angle;
}

void SpinningTarget::modify_angle(double angle)
{
    m_angle = angle;
}

double SpinningTarget::get_angular_velocity() const
{
    return m_angular_velocity;
}

void SpinningTarget::modify_angular_velocity(double angular_velocity)
{
    m_angular_velocity = angular_velocity;
}

void SpinningTarget::modify_predicted_angle(double predicted_angle)
{
    m_predicted_angle = predicted_angle;
}

void SpinningTarget::modify_last_observed_angle(double last_observed_angle)
{
    m_last_observed_angle = last_observed_angle;
}
int TrackingBase::get_id() const noexcept
{
    return m_id;
}
TargetType TrackingBase::get_type() const noexcept
{
    return m_type;
}
TargetStatus TrackingBase::get_status() const noexcept
{
    return m_status;
}
double TrackingBase::get_x() const noexcept
{
    return m_x;
}
double TrackingBase::get_y() const noexcept
{
    return m_y;
}
std::int64_t TrackingBase::get_last_observed_timestamp_ns() const noexcept
{
    return m_last_observed_timestamp_ns;
}
int TrackingBase::get_Del_count()
{
    return Del_count;
}
int TrackingBase::get_LostCount()
{
    return LostCount;
}
bool TrackingBase::get_isDelete()
{
    return isDelete;
}
bool TrackingBase::get_isChange()
{
    return isChange;
}
void TrackingBase::modify_isChange(bool a)
{
    isChange = a;
}
void TrackingBase::modify_isDelete(bool a)
{
    isDelete = a;
}

//onFrame

Result TargetManager::onFrame(const InputFrame& frame)
{
    Result res{};
    res.frame_id = frame.frame_id;
    res.accepted = false;
    for(const auto& target : m_targets)
    {
        TargetSnapshot snapshot = target->snapshot();
        res.targets.push_back(snapshot);
        if(snapshot.status == TargetStatus::TRACK)
            res.tracking_target_count++;
        if(snapshot.status == TargetStatus::LOST)
            res.lost_target_count++;
        if(snapshot.type == TargetType::ARMOR)
            res.armor_target_count++;
        if(snapshot.type == TargetType::SPINNING)
            res.spinning_target_count++;
    }
    if(m_has_successful_frame && (frame.frame_id <= m_last_success_frame_id || frame.timestamp_ns <= m_last_success_timestamp_ns))
    return res;
    for(const auto& observation : frame.observations) //负数检查
    {
        if(observation.id < 0)
            return res;
    }
    for(auto i = 0; i < frame.observations.size(); i++) //同帧重复
    {
        for(auto j = i + 1; j < frame.observations.size(); j++)
        {
            if(frame.observations[i].id == frame.observations[j].id)
                return res;
        }
    }
    for(const auto& obs : frame.observations) //类型不一致
    {
        for(const auto& tar : m_targets)
        {
            if(tar->get_id() == obs.id && tar->get_type() != obs.type)
            {
                tar->modify_isChange(true);
            }
        }
    }
    m_targets.erase(std::remove_if(m_targets.begin(),m_targets.end(),
                [](const std::unique_ptr<TrackingBase>& target)
            {
                return target->get_isChange();
            }),m_targets.end());

    double frame_dt = 0.0;
    if(m_has_successful_frame) //算时间差
    {
        frame_dt =
        (frame.timestamp_ns - m_last_success_timestamp_ns) * 1e-9;
    }
    for(const auto& obs : frame.observations) //遍历观测
    {
        bool isfind = false;
        for(const auto& tar : m_targets)
        {
            if(obs.id == tar->get_id())
            {
                tar->updateFind(obs, frame.timestamp_ns, frame_dt);
                isfind = true;
                break;
            }
        }
        if(!isfind)
        {
            if(obs.type == TargetType::ARMOR)
            {
                auto new_target = std::make_unique<ArmorTarget>(obs, frame.timestamp_ns);
                m_targets.push_back(std::move(new_target));
            }
            if(obs.type == TargetType::SPINNING)
            {
                auto new_target = std::make_unique<SpinningTarget>(obs, frame.timestamp_ns);
                m_targets.push_back(std::move(new_target));
            }

        }
    }
    for(auto &target : m_targets) //遍历已创建目标
    {
        bool isfind2 = false;
        for(auto &obs : frame.observations)
        {
            if(target->get_id() == obs.id)
            {
                isfind2 = true;
                break;
            }
        }
        if(!isfind2)
        {
            target->updateNotFind(frame_dt);
            if(target->get_Del_count() <= target->get_LostCount())
                target->modify_isDelete(true);
        }

    }
    m_targets.erase(std::remove_if(m_targets.begin(),m_targets.end(),
                    [](const std::unique_ptr<TrackingBase>& target)
                {
                    return target->get_isDelete();
                }),m_targets.end());

    res.armor_target_count = 0;
    res.lost_target_count = 0;
    res.spinning_target_count = 0;
    res.tracking_target_count = 0;
    res.targets.clear();

    for(auto &target : m_targets)
    {
        TargetSnapshot snapshot = target->snapshot();
        res.targets.push_back(snapshot);
        if(snapshot.status == TargetStatus::TRACK)
            res.tracking_target_count++;
        if(snapshot.status == TargetStatus::LOST)
            res.lost_target_count++;
        if(snapshot.type == TargetType::ARMOR)
            res.armor_target_count++;
        if(snapshot.type == TargetType::SPINNING)
            res.spinning_target_count++;
    }
    m_last_success_frame_id = frame.frame_id;
    m_last_success_timestamp_ns = frame.timestamp_ns;
    m_has_successful_frame = true;
    res.accepted = true;
    return res;
}

TargetSnapshot TrackingBase::snapshot() const //基类
{
    TargetSnapshot result{};
    result.id = m_id;
    result.status = m_status;
    result.type = m_type;
    result.x = m_x;
    result.y = m_y;
    result.velocity_x = m_velocity_x;
    result.velocity_y = m_velocity_y;
    result.predicted_x = m_predicted_x;
    result.predicted_y = m_predicted_y;
    result.missed_count = m_missed_count;
    result.angle = 0.0;
    result.angular_velocity = 0.0;
    result.predicted_angle = 0.0;

    return result;
}

void TrackingBase::updateFind(const Observation& observation,std::int64_t timestamp_ns,double frame_dt)
{
    double obs_dt = (timestamp_ns - m_last_observed_timestamp_ns) * 1e-9;
    double old_x = m_x;
    double old_y = m_y;
    if(m_status == TargetStatus::TRACK)
    {
        m_velocity_x = (observation.x - old_x) / obs_dt;
        m_velocity_y = (observation.y - old_y) / obs_dt;
    }
    if(m_status == TargetStatus::LOST)
    {
        modify_status(TargetStatus::TRACK);
        modify_missed_count(0);
        double x = get_x();
        double y = get_y();
        modify_x(x);
        modify_y(y);
        modity_velocity_x(0.0);
        modity_velocity_y(0.0);
        LostCount = 0;
    }
    m_status = TargetStatus::TRACK;
    m_x = observation.x;
    m_y = observation.y;
    m_missed_count = 0;
    m_predicted_x = m_velocity_x * frame_dt + m_x;
    m_predicted_y = m_velocity_y * frame_dt + m_y;
    m_last_observed_timestamp_ns = timestamp_ns;

}

void TrackingBase::updateNotFind(double dt)
{
    if(m_status == TargetStatus::TRACK)
    {
        m_predicted_x += m_velocity_x * dt;
        m_predicted_y += m_velocity_y * dt;
        m_missed_count++;
    }
    if(m_missed_count == lostThreshold())
    {
        m_status = TargetStatus::LOST;
    }

    if(m_status == TargetStatus::LOST)
    {
        m_x = 0.0;
        m_y = 0.0;
        m_velocity_x = 0.0;
        m_velocity_y = 0.0;
        m_predicted_x = 0.0;
        m_predicted_y = 0.0;
        LostCount++;
    }
}

TargetSnapshot SpinningTarget::snapshot() const //旋转
{
    TargetSnapshot result = TrackingBase::snapshot();
    result.angle = m_angle;
    result.angular_velocity = m_angular_velocity;
    result.predicted_angle = m_predicted_angle;
    return result;
}

void SpinningTarget::updateFind(const Observation& observation,std::int64_t timestamp_ns,double frame_dt)
{
    TargetStatus status = get_status();
    if(status == TargetStatus::LOST)
    {
        modify_angle(observation.angle);
        modify_angular_velocity(0.0);
        modify_predicted_angle(observation.angle);
        modify_last_observed_angle(observation.angle);
    }
    if(status == TargetStatus::TRACK)
    {
        double last_observed_timestamp_ns = get_last_observed_timestamp_ns();
        m_angular_velocity = (observation.angle - m_last_observed_angle)
         / (timestamp_ns - last_observed_timestamp_ns) * 1e9;
        m_predicted_angle = observation.angle + m_angular_velocity * frame_dt;
        m_angle = observation.angle;
        modify_last_observed_angle(observation.angle);
    }
    TrackingBase::updateFind(observation,timestamp_ns,frame_dt);
}

void SpinningTarget::updateNotFind(double dt)
{
    TrackingBase::updateNotFind(dt);
    TargetStatus status = get_status();
    if(status == TargetStatus::TRACK)
    {
        m_angle += m_angular_velocity * dt;
        m_predicted_angle = m_angle + m_angular_velocity * dt;
    }
    else if(status == TargetStatus::LOST)
    {
        m_angle = 0.0;
        m_predicted_angle = 0.0;
        m_angular_velocity = 0.0;
    }
}

}