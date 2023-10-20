#ifndef DELAYHZ_DEF_H__
#define DELAYHZ_DEF_H__

#include "rclcpp/rclcpp.hpp"
#include <deque>
#include <QMutex>

template <typename MessageT>
class RosDelayHz
{
private:
        struct timespec t = {0, 0};
        int messagecount = 0;
        struct timespec start_time;
        double delay = 0;
        std::deque<struct timespec> window;
        size_t window_size = 200;
        QMutex qMtuxcount, qMtuxdelay,qMtuxUpdate;

        void update(struct timespec new_value)
        {
                window.push_back(new_value);
                if (window.size() > window_size)
                {
                        window.pop_front();
                }
        }
        struct timespec getFirstValue() const
        {
                if (!window.empty())
                {
                        return window.front();
                }
                else
                {
                        return {0, 0}; // 返回适当的默认值，根据应用需要修改
                }
        }

        struct timespec getLastValue() const
        {
                if (!window.empty())
                {
                        return window.back();
                }
                else
                {
                        return {0, 0}; // 返回适当的默认值，根据应用需要修改
                }
        }

public:
        RosDelayHz(){};
        double msgDelay(const typename MessageT::ConstSharedPtr msg)
        {
                struct timespec timenow = {0, 0};
                clock_gettime(CLOCK_REALTIME, &timenow);
                if (messagecount == 0)
                {
                        clock_gettime(CLOCK_REALTIME, &start_time);
                }
                qMtuxcount.lock();
                messagecount++;
                qMtuxcount.unlock();
                std::cout << msg->header.frame_id << " delaycount:" << messagecount << std::endl;
                std::cout<<"timenow  sec"<<timenow.tv_nsec<<"   nsec"<<timenow.tv_sec<<std::endl;
                double delayt = (timenow.tv_sec + timenow.tv_nsec * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
                qMtuxdelay.lock();
                delay += delayt;
                qMtuxdelay.unlock();
                double aver_delayt = delay / messagecount;

                return aver_delayt;
        }

        double msgHz(const typename MessageT::ConstSharedPtr msg)
        {
                if (msg->header.stamp.nanosec)
                {
                        clock_gettime(CLOCK_REALTIME, &t);
                        qMtuxUpdate.lock();
                        update(t);
                        qMtuxUpdate.unlock();
                        struct timespec firsttime = getFirstValue();
                        struct timespec lasttime = getLastValue();
                        double hz = window.size() / ((lasttime.tv_sec + lasttime.tv_nsec * 1e-9) - (firsttime.tv_sec + firsttime.tv_nsec * 1e-9));
                        return hz;
                }
                throw std::runtime_error("no messages from func msgHz");
        }

        double msgDelay(MessageT* msg)
        {
                struct timespec timenow = {0, 0};
                clock_gettime(CLOCK_REALTIME, &timenow);
                if (messagecount == 0)
                {
                        clock_gettime(CLOCK_REALTIME, &start_time);
                }
                qMtuxcount.lock();
                messagecount++;
                qMtuxcount.unlock();
                std::cout << msg->header.frame_id << " delaycount:" << messagecount << std::endl;
                std::cout<<"timenow  sec"<<timenow.tv_nsec<<"   nsec"<<timenow.tv_sec<<std::endl;
                double delayt = (timenow.tv_sec + timenow.tv_nsec * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
                qMtuxdelay.lock();
                delay += delayt;
                qMtuxdelay.unlock();
                double aver_delayt = delay / messagecount;
                std::cout<<"delay"<<delay<<std::endl;

                return aver_delayt;
        }

        double msgHz(MessageT* msg)
        {
                if (msg->header.stamp.nanosec)
                {
                        clock_gettime(CLOCK_REALTIME, &t);
                        qMtuxUpdate.lock();
                        update(t);
                        qMtuxUpdate.unlock();
                        struct timespec firsttime = getFirstValue();
                        struct timespec lasttime = getLastValue();
                        double hz = window.size() / ((lasttime.tv_sec + lasttime.tv_nsec * 1e-9) - (firsttime.tv_sec + firsttime.tv_nsec * 1e-9));
                        return hz;
                }
                throw std::runtime_error("no messages from func msgHz");
        }

};

#endif