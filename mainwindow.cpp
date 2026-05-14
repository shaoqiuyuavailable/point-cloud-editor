#include "mainwindow.h"
#include "ui_mainwindow.h"
#include<QLabel>
#include<QFile>
#include<QFileDialog>
#include<vtkGenericOpenGLRenderWindow.h>
#include<vtkOpenGLFramebufferObject.h>
#include<vtkOpenGLState.h>
#include<pcl/common/transforms.h>
#include <pcl/conversions.h>
#include<Eigen/Dense>
#include<vtkoutputwindow.h>
#include<pcl/range_image/range_image.h>
#include <QColorDialog>
#include"querypointdialog.h"
#include"addpointdialog.h"
#include<cmath>
#include <QtConcurrent/QtConcurrentMap>
#include <numeric>   // for std::iota
#include<QElapsedTimer>
#include<PointMoveTask.h>
#include<QMessageBox>
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    backStack =new sizedStack<operateStep<pcl::PointXYZRGB>>(20);
    recoverStack =new sizedStack<operateStep<pcl::PointXYZRGB>>(20);
    iniUI();
    initsignal();
    setinteractor();
}
void MainWindow::initsignal(){

}
void MainWindow::iniUI(){
    //初始化底部状态栏
    pointsize=new QLabel(QStringLiteral("点云大小："),this);
    pointsize->setMinimumHeight(40);
    pointsize->setMinimumWidth(200);
    pointpos=new QLabel(QStringLiteral("选定点位置："),this);
    pointpos->setMinimumHeight(40);
    pointpos->setMinimumWidth(200);
    pointidx=new QLabel(QStringLiteral("选定点索引"),this);
    pointidx->setMinimumHeight(40);
    pointidx->setMinimumWidth(200);
    // 添加到状态栏
    ui->statusbar->addWidget(pointsize);
    ui->statusbar->addWidget(pointpos);
    ui->statusbar->addWidget(pointidx);
    // 这一版暂时没有大问题，但是会有两个窗口，展示点云的窗口独立于主窗口之外
    auto renderer=vtkSmartPointer<vtkRenderer>::New();
    auto renderwindow=vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    if (!renderer || !renderwindow) {
        qDebug() << "Renderer or RenderWindow is not initialized!" ;
        return;
    }
    renderwindow->AddRenderer(renderer);
    ui->vtkWidget->setRenderWindow(renderwindow);  // 绑定渲染窗口
    //ui->vtkWidget->setCursor(Qt::CrossCursor);
    //在头文件中，部分变量的声明
    // QVTKOpenGLNativeWidget *vtkWidget;        // VTK渲染窗口容器
    // pcl::visualization::PCLVisualizer::Ptr viewer;  // PCL可视化对象
    ui->vtkWidget->resize(700,500);
    ui->vtkWidget->setGeometry(240,40,1200,800);
    viewer.reset(new pcl::visualization::PCLVisualizer("PCL Viewer"));
    //如果不要这个独立窗口，使用PCLVisualizer("PCL Viewer",false)构造，但需解决交互器的问题
    // viewer.reset(new pcl::visualization::PCLVisualizer(render,renderwindow,"PCL viewer",false));
    // ui->vtkWidget->setRenderWindow(viewer->getRenderWindow());
    // viewer->setupInteractor(ui->vtkWidget->interactor(), ui->vtkWidget->renderWindow());
    //有问题，使用PCLVisualizer(renderer,renderwindow,"PCL Viewer", false)构造时，语法正确编译成功,但程序直接崩溃
    //当使用PCLVisualizer("PCL Viewer",false)构造时，程序正常运行，但是在导入数据后，交互时报错,确定为Interactor的问题
    //vtkOpenGLState.cxx:68    WARN| Error in cache state for GL_DEPTH_WRITEMASK
    //Generic Warning: In vtkOpenGLState.cxx, line 68 Error in cache state for GL_DEPTH_WRITEMASK
    //vtkGenericOpenGLRenderWindow (000001C685DCC460): error before running VTK rendering code 16 OpenGL errors detected
    viewer->setShowFPS(true);
    viewer->getRenderWindow()->SetSize(1200,800);
    viewer->getRenderWindow()->SetPosition(760,330);
}

void MainWindow::setinteractor(){//鼠标左键已经有绑定了许多默认操作了，使用右键进行编辑操作。
    callback = [this](const pcl::visualization::PointPickingEvent& event) {//小修小改
        pcl::index_t curpointidx = event.getPointIndex();//找到索引
        //qDebug()<<curpointidx;
        if(is_deleting){
            if(curpointidx!=-1){
                //逻辑删除（重叠点）性能更好，采用逻辑删除，还是实际删除（vector.erase）
                //？纠结啊啊啊啊啊啊
                //cloud1->points.erase(cloud1->points.begin()+curpointidx);吃性能
                operateStep<pcl::PointXYZRGB> opt;
                std::pair<pcl::index_t,pcl::PointXYZRGB> tempair;
                tempair.first=curpointidx;
                tempair.second=cloud1->points[curpointidx];
                opt.operates.push_back(tempair);
                backStack->push(opt);
                cloud1->points[curpointidx]=cloud1->points[0];
                viewer->updatePointCloud(cloud1,"cloud1");
            }
        }
        else{
            selectedPoints.clear();
            viewer->removePointCloud("highlighted_cloud");
            if(curpointidx != -1) {
                // 先移除可能存在的旧标记
                highlight_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
                // 获取点的坐标
                float x, y, z;
                event.getPoint(x, y, z);
                // 从点云中获取该点（验证坐标一致性）
                pcl::PointXYZRGB selected_point = cloud1->points[curpointidx];
                qDebug() << "Cloud point:" << selected_point.x << selected_point.y << selected_point.z;
                // 高亮显示选中的点（红色+加粗）
                // 创建只包含选中点的点云
                highlight_cloud->push_back(selected_point);
                // 添加高亮点云（红色，点尺寸加大）
                viewer->addPointCloud(highlight_cloud, "highlighted_cloud");
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "highlighted_cloud");
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "highlighted_cloud");
                // 刷新视图
                viewer->getRenderWindow()->Render();
                selected=true;
                selectedPoints.push_back(curpointidx);
                pointidx->setText(QStringLiteral("选定点索引:")+QString::number(curpointidx));
                pointpos->setText(QStringLiteral("选定点位置:")+QString::number(selected_point.x)+","+QString::number(selected_point.y)+","+QString::number(selected_point.z));
            }
        }
    };
    viewer->registerPointPickingCallback(callback);
    callback1=[this](const pcl::visualization::AreaPickingEvent&event){//区域内选点,并做标记
        selectedPoints.clear();
        selectedPoints=event.getPointsIndices("cloud1");
        viewer->removePointCloud("highlighted_cloud");
        //qDebug()<<selectedPoints.size();
        if(selectedPoints.size()>0){
            highlight_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            for(int i=0;i<selectedPoints.size();i++){
                highlight_cloud->points.push_back(cloud1->points[selectedPoints[i]]);
            }
            viewer->addPointCloud(highlight_cloud, "highlighted_cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "highlighted_cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "highlighted_cloud");
            // 刷新视图
            viewer->getRenderWindow()->Render();
            selected=true;
        }
    };
    viewer->registerAreaPickingCallback(callback1);
    keyboardcallback=[this](const pcl::visualization::KeyboardEvent &event){
        if(selected){//如果有点被选中
            static int key_code;
            //每次位移的距离根据原数据cloud2决定，初步定位+=原始坐标坐标的5%，还可设置单次移动的比例参数
            int a=0;
            int b=0;
            int c=0;
            if(event.keyDown()){
                key_code = event.getKeyCode();
                //一个for循环，先将为发生改变的点记录下来
                operateStep<pcl::PointXYZRGB> temp;
                for(int i=0;i<selectedPoints.size();i++){
                    std::pair<pcl::index_t,pcl::PointXYZRGB> tempair;
                    tempair.first=selectedPoints[i];
                    tempair.second=cloud1->points[selectedPoints[i]];
                    temp.operates.push_back(tempair);
                }
                if(key_code=='w'||key_code=='W'){
                    for(int i=0;i<selectedPoints.size();i++){
                        double x=cloud2->points[selectedPoints[i]].x;
                        cloud1->points[selectedPoints[i]].x+=x*0.05;
                    }
                    a++;
                }
                if(key_code=='e'||key_code=='E'){
                    for(int i=0;i<selectedPoints.size();i++){
                        double x=cloud2->points[selectedPoints[i]].x;
                        cloud1->points[selectedPoints[i]].x-=x*0.05;
                    }
                    a--;
                }
                if(key_code=='a'||key_code=='A'){
                    for(int i=0;i<selectedPoints.size();i++){
                        double y=cloud2->points[selectedPoints[i]].y;
                        cloud1->points[selectedPoints[i]].y+=y*0.05;
                    }
                    b++;
                }
                if(key_code=='s'||key_code=='S'){
                    for(int i=0;i<selectedPoints.size();i++){
                        double y=cloud2->points[selectedPoints[i]].y;
                        cloud1->points[selectedPoints[i]].y-=y*0.05;
                    }
                    b--;
                }
                if(key_code=='z'||key_code=='Z'){
                    for(int i=0;i<selectedPoints.size();i++){
                        double z=cloud2->points[selectedPoints[i]].z;
                        cloud1->points[selectedPoints[i]].z+=z*0.05;
                    }
                    c++;
                }
                if(key_code=='v'||key_code=='V'){
                    for(int i=0;i<selectedPoints.size();i++){
                        double z=cloud2->points[selectedPoints[i]].z;
                        cloud1->points[selectedPoints[i]].z-=z*0.05;
                    }
                    c--;
                }
                viewer->updatePointCloud(cloud1,"cloud1");
                //同步移动高光点
                for(int i=0;i<highlight_cloud->points.size();i++){
                    double x=highlight_cloud->points[i].x;
                    double y=highlight_cloud->points[i].y;
                    double z=highlight_cloud->points[i].z;
                    //qDebug()<<x<<" "<<y<<" "<<z;
                    //qDebug()<<a<<" "<<b<<" "<<c;
                    highlight_cloud->points[i].x+=a*x*0.05;  
                    highlight_cloud->points[i].y+=b*y*0.05;
                    highlight_cloud->points[i].z+=c*z*0.05;
                    //qDebug()<<highlight_cloud->points[i].x<<" "<<highlight_cloud->points[i].y<<" "<<highlight_cloud->points[i].z;
                }
                viewer->updatePointCloud(highlight_cloud,"highlighted_cloud");
                //将这些点和操作全部压入操作栈
                backStack->push(temp);
            }
        }
    };
    viewer->registerKeyboardCallback(keyboardcallback);
    mousemoveCallback=[this](const pcl::visualization::MouseEvent&event){
        //从 selectedPoints中选出被选择点的索引
        static int startX, startY;        // 起始屏幕坐标
        static pcl::Indices operateidx;
        static std::vector<pcl::PointXYZRGB> prepoints;
        static operateStep<pcl::PointXYZRGB> temp;
        //static std::vector<std::pair<pcl::index_t,pcl::PointXYZ>> pointpair;
        if (event.getType() == pcl::visualization::MouseEvent::MouseButtonPress &&
            event.getButton() == pcl::visualization::MouseEvent::RightButton &&selected){ // 确保有选中的点
            is_dragging = true;  // 设置拖拽状态
            // 记录起始位置的信息,即被选中的点
            operateidx=selectedPoints;
            for(int i=0;i<selectedPoints.size();i++){
                prepoints.push_back(cloud1->points[selectedPoints[i]]);
            }
            //qDebug()<<"prepoint"<<prepoint.x<<" "<<prepoint.y<<" "<<prepoint.z;
            startX=event.getX();
            startY=event.getY();
            //qDebug()<<"start: "<<startX<<" "<<startY;
        }
        if (event.getType() == pcl::visualization::MouseEvent::MouseMove &&is_dragging){
            //拖拽过程中进行移动，动态更新点的位置
            //注意性能
            //qDebug()<<"move";
        }
        if (event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease &&
            event.getButton() == pcl::visualization::MouseEvent::RightButton&&is_dragging){

            QElapsedTimer timer;
            timer.start();
            int endX = event.getX();
            int endY = event.getY();
            // 获取相机参数和位移向量（与您原有代码一致）
            pcl::visualization::Camera camera;
            viewer->getCameraParameters(camera);
            // double fov = camera.fovy;
            double outdepth = (camera.window_size[1] / 2) / tan(15 * M_PI / 180.0);
            Eigen::Vector3d view_dir(camera.focal[0] - camera.pos[0],
                                     camera.focal[1] - camera.pos[1],
                                     camera.focal[2] - camera.pos[2]);
            view_dir.normalize();
            Eigen::Vector3d up_dir(camera.view[0], camera.view[1], camera.view[2]);
            up_dir.normalize();
            Eigen::Vector3d right_dir = up_dir.cross(view_dir);
            right_dir.normalize();
            Eigen::Matrix3d R1;
            R1.col(0) = -right_dir;
            R1.col(1) = up_dir;
            R1.col(2) = view_dir;
            Eigen::Vector3d displace(endX - startX, endY - startY, 0);
            Eigen::Vector3d acturedisplace = R1 * displace;
            int totalPoints = static_cast<int>(selectedPoints.size());
            if (totalPoints == 0) {
                is_dragging = false;
                return;
            }
            const int BATCH_SIZE = 1000;
            int numTasks = (totalPoints + BATCH_SIZE - 1) / BATCH_SIZE;
            QAtomicInt* remainingTasks = new QAtomicInt(numTasks);   // 堆分配，跨线程共享
            for (int t = 0; t < numTasks; ++t) {
                int start = t * BATCH_SIZE;
                int end = std::min(start + BATCH_SIZE, totalPoints);
                PointMoveTask* task = new PointMoveTask(start, end,selectedPoints,
                                                        cloud1,highlight_cloud,
                                                        camera,view_dir,
                                                        acturedisplace,outdepth,
                                                        remainingTasks,this);
                QThreadPool::globalInstance()->start(task);
            }
            // 更新视图
            // qDebug() << "time taken:" << timer.elapsed();
            viewer->updatePointCloud(cloud1, "cloud1");
            viewer->updatePointCloud(highlight_cloud, "highlighted_cloud");
            viewer->getRenderWindow()->Render();
            qDebug() << "time taken:" << timer.elapsed();
            qDebug() << "All tasks finished, view updated.";//
            //记录操作结果，将其放入操作栈中
            //存放信息包括了被操作点的序号，操作前的位置信息，不需要操作后的位置信息，通过序号直接在点云中进行寻找

            //改为支持多个点的移动
            //似乎有点问题,撤销时似乎把点撤掉了
            for(int i=0;i<operateidx.size();i++){
                std::pair<pcl::index_t,pcl::PointXYZRGB> tempair;
                tempair.first=operateidx[i];
                tempair.second=prepoints[i];
                temp.operates.push_back(tempair);
            }
            //qDebug()<<operateidx.size();
            backStack->push(temp);
            //清除中间数据，重置状态，需要吗？
            //viewer->removePointCloud("highlighted_cloud");
            //清除高光点还是同步移动高光点
            //若同步移动，则不应该清空高光点，也不应该清空被选择点，selected始终为true，仅is_dragging变为false;
            //若清除高光点，则应该清空operateidx,prepoints,selectedPoints
            prepoints.clear();
            operateidx.clear();
            is_dragging=false;
        }
    };
    viewer->registerMouseCallback(mousemoveCallback); //再重写shift加左键移位的函数，将其改变为对选定渲染点的移动操作而不是整体点云的移动
}

MainWindow::~MainWindow()
{
    delete ui;
    if(viewer){
        //on_actionclear_triggered();
        viewer->close();
    }
}
QString MainWindow::getFile(bool save){//获取文件名
    QString filename;
    if(save){
        filename=QFileDialog::getSaveFileName(this,QStringLiteral("请选择文件"),".","*.pcd");
    }
    else
        filename=QFileDialog::getOpenFileName(this,QStringLiteral("请选择文件"),".","*.pcd");
    return filename;
}
void MainWindow::on_actionopen_triggered()
{
    cloud.reset(new pcl::PCLPointCloud2);
    QString filepath=getFile(false);
    //打开
    if(filepath.isEmpty()){
        //预留提示，暂不实现
    }
    else{
        pcl::io::loadPCDFile(filepath.toStdString(),*cloud);//加载点云
        cloud1.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud2.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(*cloud,*cloud2);//出错，弹出越界错误，初步认定为数据量过大，246MB
        pcl::fromPCLPointCloud2<pcl::PointXYZRGB>(*cloud,*cloud1);
        pcl::getMinMax3D(*cloud1,minPt, maxPt);//获取点云的基本数据
        //center=new Eigen::Vector3f((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
        viewer->addPointCloud(cloud1,"cloud1");
        viewer->resetCamera();
        viewer->getRenderWindow()->Render();
        int k=cloud1->points.size();
        this->pointsize->setText(QStringLiteral("点云大小：")+QString::number(k));
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1.0,1.0,1.0,"cloud1");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud1");

    }
}

void MainWindow::on_actionsave_triggered()
{
    QString filepath=getFile(true);
    if(!filepath.isEmpty()){
        pcl::PCDWriter writer;
        //将cloud进行写入，则对cloud1（渲染数据）的操作通过变换同步到cloud中
        //对cloud1进行transfrom矩阵的逆变换，
        //先将副本数据cloud2进行仿射逆变换
        Eigen::Affine3f antitransform =transform.inverse();
        pcl::transformPointCloud(*cloud1,*cloud2,antitransform);
        //再将cloud2的数据输入到cloud原始数据中，再保存
        pcl::toPCLPointCloud2(*cloud2,*cloud);
        writer.write(filepath.toStdString(),*cloud);
    }
}

void MainWindow::on_actionclear_triggered()
{
    //检查指针是否为空 //清除数据
    if(cloud){
        cloud1.reset();
        backStack->clear();
        recoverStack->clear();
        cloud2.reset();
    }
    //渲染重新渲染
    viewer->setBackgroundColor(0.0,0.0,0.0);
    viewer->removeAllPointClouds();
}

//今日任务，修复撤销操作导致点丢失的bug
void MainWindow::on_actionback_triggered(){
    //回退操作，并将回退的操作放入recover栈中,吞数据
    if(!backStack->container->empty()){
        operateStep<pcl::PointXYZRGB> *top=backStack->getTop();
        operateStep<pcl::PointXYZRGB> temp;
        for(int i=0;i<top->operates.size();i++){
            std::pair<pcl::index_t,pcl::PointXYZRGB> tempair;
            tempair.first=top->operates[i].first;
            tempair.second=cloud1->points[tempair.first];
            temp.operates.push_back(tempair);
        }
        for(int i=0;i<temp.operates.size();i++){
            pcl::index_t preidx=top->operates[i].first;
            cloud1->points[preidx]=top->operates[i].second;
        }
        viewer->updatePointCloud(cloud1,"cloud1");
        viewer->getRenderWindow()->Render();
        recoverStack->push(temp);
        backStack->pop();
    }
}

void MainWindow::on_actionrecover_triggered()
{
    //恢复操作，将操作放入back栈中
    if(!recoverStack->container->empty()){
        operateStep<pcl::PointXYZRGB> *top=recoverStack->getTop();//目前仅支持单点回退操作
        operateStep<pcl::PointXYZRGB> temp;
        for(int i=0;i<top->operates.size();i++){
            std::pair<pcl::index_t,pcl::PointXYZRGB> tempair;
            tempair.first=top->operates[i].first;
            tempair.second=cloud1->points[tempair.first];
            temp.operates.push_back(tempair);
        }
        for(int i=0;i<top->operates.size();i++){
            pcl::index_t preidx=top->operates[i].first;
            cloud1->points[preidx]=top->operates[i].second;
        }
        viewer->updatePointCloud(cloud1,"cloud1");
        viewer->getRenderWindow()->Render();
        backStack->push(temp);
        recoverStack->pop();
    }
}


void MainWindow::on_actionclose_triggered()
{
    this->close();
}

void MainWindow::movecamera(){
    int x=ui->horizontalSlider_x->value();
    int y=ui->horizontalSlider_y->value();
    int z=ui->horizontalSlider_z->value();
    float xdis=maxPt.x-minPt.x;
    float ydis=maxPt.y-minPt.y;
    float zdis=maxPt.z-minPt.z;
    float k=0.01;
    float tx = x*xdis*k;
    float ty=y*k*ydis;
    float tz=z*k*zdis;
    transform.translation()<<tx,ty,tz;
    pcl::transformPointCloud(*cloud2,*cloud1,transform);
    viewer->updatePointCloud(cloud1,"cloud1");
    viewer->getRenderWindow()->Render();
}

void MainWindow::on_horizontalSlider_x_actionTriggered(int action)
{ //第四版，修复了其他方向的移位bug，但性能几乎没有下降，留到迭代二继续优化性能
    movecamera();
}

void MainWindow::on_horizontalSlider_y_actionTriggered(int action)
{
    movecamera();
}

void MainWindow::on_horizontalSlider_z_actionTriggered(int action)
{
    movecamera();
}

void MainWindow::on_Coordinatevisable_clicked(bool checked)
{
    if(checked==true){
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();//初始化变换矩阵为单位矩阵
        viewer->addCoordinateSystem(0.5,transform,"s1");//参数：缩放比例
        viewer->getRenderWindow()->Render();
    }
    else{
        viewer->removeCoordinateSystem("s1");
        viewer->getRenderWindow()->Render();  //重新渲染，渲染页面
    }
}

void MainWindow::on_resetviewer_clicked()
{
    transform.translation()<<0.0,0.0,0.0;
    ui->horizontalSlider_x->setValue(0);
    ui->horizontalSlider_y->setValue(0);
    ui->horizontalSlider_z->setValue(0);
    pcl::transformPointCloud(*cloud2,*cloud1,transform);
    viewer->updatePointCloud(cloud1,"cloud1");
    viewer->getRenderWindow()->Render();
}

void MainWindow::on_changebgc_clicked()
{
    //呼出调色版，获取用户选择颜色的三原色的值
    // 创建颜色对话框并获取用户选择的颜色
    QColor color = QColorDialog::getColor(Qt::white, this,QStringLiteral("选择背景颜色"));
    // 检查用户是否选择了有效颜色（非取消操作）
    if (color.isValid()) {
        // 将QColor的0-255范围转换为PCL所需的0.0-1.0范围
        double r = color.redF();
        double g = color.greenF();
        double b = color.blueF();
        // 设置PCL可视化窗口的背景色
        viewer->setBackgroundColor(r, g, b);
        // 强制刷新渲染窗口
        viewer->getRenderWindow()->Render();
    }
}

void MainWindow::on_pointquery_clicked()//弹出窗口，用户在窗口查询点
{
    querypointDialog*dialog=new querypointDialog(this);
    dialog->setWindowFlag(Qt::MSWindowsFixedSizeDialogHint);
    int ret=dialog->exec();
    if(ret==QDialog::Accepted){
        //怎么查询，查询什么，留待之后考虑
        delete dialog;
    }
}

void MainWindow::on_deletemode_clicked(bool checked)
{
    if(checked){
        is_deleting=true;
    }
    else{
        is_deleting=false;
    }
}

void MainWindow::on_addpoint_triggered()//弹出窗口，用户在窗口录入添加点的信息
{
    addpointDialog* dialog=new addpointDialog(this);
    dialog->setWindowFlag(Qt::MSWindowsFixedSizeDialogHint);
    int ret=dialog->exec();
    if(ret==QDialog::Accepted){
        if(dialog->px->text().isEmpty()||dialog->py->text().isEmpty()||dialog->pz->text().isEmpty()){
            //进行提示
        }
        else{
            pcl::PointXYZRGB newpoint(dialog->px->text().toFloat(),dialog->py->text().toFloat(),dialog->pz->text().toFloat());
            //pcl::PointXYZRGB  newpoint()
            //qDebug()<<newpoint.x;
            if(cloud1){
                //将点放入操作栈中。
                operateStep<pcl::PointXYZRGB> opt;
                std::pair<pcl::index_t,pcl::PointXYZRGB> tempair;
                tempair.first=cloud1->points.size();
                tempair.second=newpoint;
                opt.operates.push_back(tempair);
                backStack->push(opt);
                cloud1->points.push_back(newpoint);
                viewer->updatePointCloud(cloud1,"cloud1");

            }
            else{
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp;
                temp.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
                temp->points.push_back(newpoint);
                viewer->addPointCloud(temp,"temp");
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "temp");
                viewer->setPointCloudRenderingProperties(
                    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "temp");

                viewer->getRenderWindow()->Render();
            }
            delete dialog;
        }

    }
}

void MainWindow::on_objedge_clicked()//物体边界识别
{
    // 创建深度图像
    // 创建深度图像
    float angularResolutionX = static_cast<float>(1.0f * (M_PI / 180.0f));
    float angularResolutionY = static_cast<float>(1.0f * (M_PI / 180.0f));  // 垂直分辨率 (1度)
    float maxAngleWidth = static_cast<float>(360.0f * (M_PI / 180.0f));
    float maxAngleHeight = static_cast<float>(180.0f * (M_PI / 180.0f));
    Eigen::Affine3f sensorPose = Eigen::Affine3f::Identity();
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    // 使用 std::shared_ptr 替代 boost::shared_ptr
    //std::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    //pcl::RangeImage& rangeImage = *range_image_ptr;//创建深度图像用于边缘检测
    pcl::RangeImage rangeImage;
    qDebug()<<1;

    rangeImage.createFromPointCloud(*cloud1, angularResolutionX,angularResolutionY, maxAngleWidth,//有问题，卡死,崩溃,参数错误
                                    maxAngleHeight, sensorPose, coordinate_frame,//源码133行points.clear()报错
                                    noiseLevel, minRange, borderSize);
    qDebug()<<2;
    // 修复点云颜色处理器 - 使用 std::shared_ptr 并转换为基类指针
    // pcl::PointCloud<pcl::PointWithRange>::Ptr base_cloud_ptr =
    //     std::static_pointer_cast<pcl::PointCloud<pcl::PointWithRange>>(range_image_ptr);
    // qDebug()<<3;
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange>
    //     range_image_color_handler(base_cloud_ptr, 0, 0, 0);
    // qDebug()<<4;
    // viewer->addPointCloud(base_cloud_ptr, range_image_color_handler, "range image");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "range image");

    // // ============== 边界识别部分 ============== //
    // // 提取边界特征
    // pcl::RangeImageBorderExtractor border_extractor;
    // pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    // border_extractor.compute(rangeImage, border_descriptions);

    // // 创建点云存储不同类型边界点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_borders(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr veil_points(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr shadow_points(new pcl::PointCloud<pcl::PointXYZ>);

    // obstacle_borders->points.reserve(border_descriptions.points.size());
    // veil_points->points.reserve(border_descriptions.points.size());
    // shadow_points->points.reserve(border_descriptions.points.size());

    // // 遍历所有边界点
    // for (int y = 0; y < static_cast<int>(rangeImage.height); ++y) {
    //     for (int x = 0; x < static_cast<int>(rangeImage.width); ++x) {
    //         // 跳过无效点
    //         if (!pcl::isFinite(rangeImage.getPoint(x, y))) continue;

    //         pcl::PointWithRange point = rangeImage.getPoint(x, y);
    //         pcl::BorderTraits traits = border_descriptions.points[y*rangeImage.width + x].traits;

    //         if (traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]) {
    //             obstacle_borders->points.emplace_back(point.x, point.y, point.z);
    //         }
    //         if (traits[pcl::BORDER_TRAIT__VEIL_POINT]) {
    //             veil_points->points.emplace_back(point.x, point.y, point.z);
    //         }
    //         if (traits[pcl::BORDER_TRAIT__SHADOW_BORDER]) {
    //             shadow_points->points.emplace_back(point.x, point.y, point.z);
    //         }
    //     }
    // }

    // // 设置点云属性
    // obstacle_borders->width = obstacle_borders->points.size();
    // obstacle_borders->height = 1;
    // obstacle_borders->is_dense = false;

    // veil_points->width = veil_points->points.size();
    // veil_points->height = 1;
    // veil_points->is_dense = false;

    // shadow_points->width = shadow_points->points.size();
    // shadow_points->height = 1;
    // shadow_points->is_dense = false;

    // // 可视化边界点
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> obstacle_color(obstacle_borders, 255, 0, 0);   // 红色: 障碍边界
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> veil_color(veil_points, 0, 255, 0);          // 绿色: 远边界
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> shadow_color(shadow_points, 0, 0, 255);       // 蓝色: 阴影边界

    // viewer->addPointCloud(obstacle_borders, obstacle_color, "obstacle borders");
    // viewer->addPointCloud(veil_points, veil_color, "veil points");
    // viewer->addPointCloud(shadow_points, shadow_color, "shadow points");

    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "obstacle borders");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "veil points");
    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "shadow points");
}
