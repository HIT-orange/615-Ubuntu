#ifndef CURVES_H
#define CURVES_H

#include <QWidget>

namespace Ui {
class curves;
}

class curves : public QWidget
{
    Q_OBJECT

public:
    explicit curves(QWidget *parent = nullptr);
    ~curves();

private:
    Ui::curves *ui;
};

#endif // CURVES_H
