#ifndef POINTMOVETASK_H
#define POINTMOVETASK_H
#include<pcl/io/pcd_io.h>
#include<pcl/common/common.h>
#include<pcl/common/transforms.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/point_cloud.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl/types.h>

class PointMoveTask : public QRunnable {
public:
    //批处理：处理 selectedPoints 中下标 [startIdx, endIdx) 的所有点
    PointMoveTask(int startIdx, int endIdx,
                  const std::vector<int>& selectedPoints,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr highlight_cloud,
                  pcl::visualization::Camera camera,
                  Eigen::Vector3d view_dir,
                  Eigen::Vector3d acturedisplace,
                  double outdepth,
                  QAtomicInt* counter,
                  QObject* notifier)
        : m_startIdx(startIdx),
        m_endIdx(endIdx),
        m_selectedPoints(selectedPoints),
        m_cloud1(cloud1),
        m_highlight_cloud(highlight_cloud),
        m_camera(camera),
        m_view_dir(view_dir),
        m_acturedisplace(acturedisplace),
        m_outdepth(outdepth),
        m_counter(counter),
        m_notifier(notifier)
    {
        setAutoDelete(true);
    }

    void run() override {
        // 批量处理这个区间内的所有点
        for (int idx = m_startIdx; idx < m_endIdx; ++idx) {
            int globalIdx = m_selectedPoints[idx];
            pcl::PointXYZRGB standardpoint = m_cloud1->points[globalIdx];
            double innerdepth = std::fabs((m_camera.pos[0] - standardpoint.x) * m_view_dir.x() +
                                          (m_camera.pos[1] - standardpoint.y) * m_view_dir.y() +
                                          (m_camera.pos[2] - standardpoint.z) * m_view_dir.z());
            Eigen::Vector3d displacein3D = m_acturedisplace * (innerdepth / m_outdepth);

            m_cloud1->points[globalIdx].x += displacein3D.x();
            m_cloud1->points[globalIdx].y += displacein3D.y();
            m_cloud1->points[globalIdx].z += displacein3D.z();

            m_highlight_cloud->points[idx].x += displacein3D.x();
            m_highlight_cloud->points[idx].y += displacein3D.y();
            m_highlight_cloud->points[idx].z += displacein3D.z();
        }

        // 最后一个任务完成时通知主线程
        if (m_counter->fetchAndAddOrdered(-1) == 1) {
            QMetaObject::invokeMethod(m_notifier, "onDragMoveFinished", Qt::QueuedConnection);
        }
    }

private:
    int m_startIdx, m_endIdx;
    const std::vector<int>& m_selectedPoints;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_highlight_cloud;
    pcl::visualization::Camera m_camera;
    Eigen::Vector3d m_view_dir;
    Eigen::Vector3d m_acturedisplace;
    double m_outdepth;
    QAtomicInt* m_counter;
    QObject* m_notifier;
};




#endif // POINTMOVETASK_H
