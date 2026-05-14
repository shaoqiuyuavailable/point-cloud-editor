#ifndef ADDPOINTDIALOG_H
#define ADDPOINTDIALOG_H

#include <QDialog>
#include<QLineEdit>
namespace Ui {
class addpointDialog;
}

class addpointDialog : public QDialog
{
    Q_OBJECT

public:
    explicit addpointDialog(QWidget *parent = nullptr);
    ~addpointDialog();
    QLineEdit *px=nullptr;
    QLineEdit *py=nullptr;
    QLineEdit *pz=nullptr;
    void initUI();

private:
    Ui::addpointDialog *ui;
};

#endif // ADDPOINTDIALOG_H
