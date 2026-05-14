#include "addpointdialog.h"
#include "ui_addpointdialog.h"

addpointDialog::addpointDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::addpointDialog)
{
    ui->setupUi(this);
    initUI();
}
void addpointDialog::initUI(){
    px=new QLineEdit(this);
    py=new QLineEdit(this);
    pz=new QLineEdit(this);
    px->setGeometry(180,70,113,20);
    py->setGeometry(180,120,113,20);
    pz->setGeometry(180,170,113,20);
}

addpointDialog::~addpointDialog()
{
    delete ui;
}
