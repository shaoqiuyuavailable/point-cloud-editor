#ifndef QUERYPOINTDIALOG_H
#define QUERYPOINTDIALOG_H

#include <QDialog>

namespace Ui {
class querypointDialog;
}

class querypointDialog : public QDialog
{
    Q_OBJECT

public:
    explicit querypointDialog(QWidget *parent = nullptr);
    ~querypointDialog();

private:
    Ui::querypointDialog *ui;
};

#endif // QUERYPOINTDIALOG_H
