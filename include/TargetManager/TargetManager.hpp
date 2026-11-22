#pragma once

#include "data_types.hpp"

#include <cstdint>
#include <memory>
#include <vector>
#include <algorithm>

namespace rm
{

class TrackingBase //基类
{
public:
    //bool isDelete = false;
    //bool isChange = false;
    TrackingBase(int id, TargetType type, double x, double y, std::int64_t timestamp_ns);
    virtual ~TrackingBase();
    int get_id() const noexcept;
    TargetType get_type() const noexcept;
    TargetStatus get_status() const noexcept;
    double get_x() const noexcept;
    double get_y() const noexcept;
    std::int64_t get_last_observed_timestamp_ns() const noexcept;

    bool get_isDelete();
    bool get_isChange();
    void modify_isChange(bool a);
    void modify_isDelete(bool a);
    void modify_type(TargetType type);
    void modify_status(TargetStatus status);
    void modify_missed_count(int count);
    void modify_x(double x);
    void modify_y(double y);
    void modity_velocity_x(double velocity_x);
    void modity_velocity_y(double velocity_y);
    int get_Del_count();
    int get_LostCount();
    virtual int lostThreshold() const noexcept = 0; //判断为lost所需的数量
    virtual TargetSnapshot snapshot() const;
    virtual void updateNotFind(double frame_dt);
    virtual void updateFind(const Observation& observation,std::int64_t timestamp_ns,double frame_dt);
private:
    int m_id;
    TargetType m_type;
    TargetStatus m_status;
    double m_x;
    double m_y;
    double m_velocity_x = 0.0;
    double m_velocity_y = 0.0;
    double m_predicted_x;
    double m_predicted_y;
    int m_missed_count = 0;
    std::int64_t m_last_observed_timestamp_ns;
    int LostCount = 0;
    static constexpr int Del_count = 10;
    bool isDelete = false;
    bool isChange = false;
};

class ArmorTarget final : public TrackingBase //普通装甲
{
public:
    ArmorTarget(const Observation& observation, std::int64_t timestamp_ns);
    int lostThreshold() const noexcept override;
private:
    static constexpr int lostArmor = 3;
};

class SpinningTarget final : public TrackingBase //旋转装甲
{
public:
    SpinningTarget(const Observation& observation, std::int64_t timestamp_ns);
    int lostThreshold() const noexcept override;
    double get_angle() const noexcept;
    void modify_angle(double angle);
    double get_angular_velocity() const;
    void modify_angular_velocity(double angular_velocity);
    void modify_predicted_angle(double predicted_angle);
    void modify_last_observed_angle(double last_observed_angle);
    TargetSnapshot snapshot() const override;
    void updateNotFind(double frame_dt) override;
    void updateFind(const Observation& observation,std::int64_t timestamp_ns,double frame_dt) override;
    static constexpr int get_lostSpinning() noexcept { return lostSpinning; }
private:
    double m_angle = 0.0;
    double m_angular_velocity = 0.0;
    double m_predicted_angle = 0.0;
    double m_last_observed_angle = 0.0; //真实角度
    static constexpr int lostSpinning = 5;
};

class TargetManager //目标管理器
{
public:
    rm::Result onFrame(const InputFrame& frame);

private:
    bool m_has_successful_frame = false;
    int m_last_success_frame_id = -1;
    std::int64_t m_last_success_timestamp_ns = 0;
    std::vector<std::unique_ptr<TrackingBase>> m_targets;//指针
};

} // namespace rm
