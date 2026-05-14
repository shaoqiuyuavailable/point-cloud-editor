#include "querypointdialog.h"
#include "ui_querypointdialog.h"

querypointDialog::querypointDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::querypointDialog)
{
    ui->setupUi(this);
}

querypointDialog::~querypointDialog()
{
    delete ui;
}
