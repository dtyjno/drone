#pragma once

// 观察者接口，用于接收位置数据更新

class PosDataObserverInterface {
public:
    virtual ~PosDataObserverInterface() = default;
    virtual void on_position_update(double x, double y, double z) = 0;
    virtual void on_velocity_update(double vx, double vy, double vz) = 0;
    virtual void on_orientation_update(double roll, double pitch, double yaw) = 0;
};

class PosDataObservable {
public:
    std::vector<PosDataObserverInterface*> observers;
    void add_observer(PosDataObserverInterface* observer) {
        observers.push_back(observer);
    }

    void remove_observer(PosDataObserverInterface* observer) {
        observers.erase(std::remove(observers.begin(), observers.end(), observer), observers.end());
    }
protected:
    void notify_position_update(double x, double y, double z) {
        for (auto observer : observers) {
            observer->on_position_update(x, y, z);
        }
    }
    void notify_velocity_update(double vx, double vy, double vz) {
        for (auto observer : observers) {
            observer->on_velocity_update(vx, vy, vz);
        }
    }
    void notify_orientation_update(double roll, double pitch, double yaw) {
        for (auto observer : observers) {
            observer->on_orientation_update(roll, pitch, yaw);
        }
    }
};