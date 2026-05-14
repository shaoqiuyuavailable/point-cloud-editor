#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include"sizedstack.h"
#include<QLabel>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl/types.h>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include<Eigen/Dense>
#include<pcl/io/pcd_io.h>
#include<pcl/common/common.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/point_cloud.h>
#include<vtkSmartPointer.h>
#include<QVTKOpenGLNativeWidget.h>
#include<QString>
#include <vtkOpenGLFramebufferObject.h>
#include<QDebug>
#include"operatestep.h"
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    QLabel*pointsize=nullptr;
    QLabel*pointpos=nullptr;
    QLabel*pointidx=nullptr;
    pcl::visualization::PCLVisualizer::Ptr viewer;  // PCL可视化对象
    pcl::PCLPointCloud2::Ptr cloud;//坐标信息以float类型最常见，迭代一中暂且以float类型表示数据,原始实际数据
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud1;//用于渲染的点云数据,可以进行仿射变换
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud2;//实际数据的副本
    //用于放射变换的矩阵
    // 1. 创建变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();//从实际数据到渲染数据的变换矩阵
//用户实际上是对渲染数据进行改变，当进行保存时，对could1进行逆变换即可得到原始数据，对点的更改也在逆变换的过程中同步到原始数据中

    QString getFile(bool save=false);
    //对点的修改情况
    sizedStack<operateStep<pcl::PointXYZRGB>> *backStack=nullptr;//暂时以PointAYZRGBA
    sizedStack<operateStep<pcl::PointXYZRGB>> *recoverStack=nullptr;
   //完成一次对点的修改后，将修改信息存放至栈中


    void iniUI();
    void initsignal();
    // 获取点云边界框大小
    pcl::PointXYZRGB minPt, maxPt;
    //Eigen::Vector3f *center;//点云中心点


    void movecamera();
    void setinteractor();
    bool selected=false;//被选择状态
    bool is_dragging = false; //拖拽状态
    bool is_deleting=false;//删除状态
    std::vector<pcl::index_t> selectedPoints;//存放被选择点的
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr highlight_cloud;

    std::function<void(const pcl::visualization::PointPickingEvent&)> callback;//需要设置一个状态变量，判断点是否被选取
    std::function<void(const pcl::visualization::AreaPickingEvent&)> callback1;//区域选点
    std::function<void(const pcl::visualization::KeyboardEvent&)> keyboardcallback;//键盘对点进行移动
    std::function<void(const pcl::visualization::MouseEvent &)> mousemoveCallback;//鼠标对点进行移动
    //判断点的选择状态，如果点被选取，通过按住shift和鼠标移位进行编辑操作，对点移位，取代原先的移动相机，如果wei
    //通过vtkInteractorStyle对点进行编辑操作
private slots:
    void on_actionopen_triggered();

    void on_actionsave_triggered();

    void on_actionclear_triggered();

    void on_actionback_triggered();

    void on_actionrecover_triggered();

    void on_actionclose_triggered();

    void on_horizontalSlider_x_actionTriggered(int action);

    void on_horizontalSlider_y_actionTriggered(int action);

    void on_horizontalSlider_z_actionTriggered(int action);

    void on_Coordinatevisable_clicked(bool checked);

    void on_resetviewer_clicked();

    void on_changebgc_clicked();

    void on_pointquery_clicked();

    void on_deletemode_clicked(bool checked);

    void on_addpoint_triggered();

    void on_objedge_clicked();

private:
    Ui::MainWindow *ui;
signals:
    //先对点进行选择，并作为信号进行发送，

};
#endif // MAINWINDOW_H
