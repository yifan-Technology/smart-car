#include "mainwindow.h"

#include <QApplication>

#include <QDebug>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;

//    const char send_info[] = {0x55, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x66};

//    float * data_ptr_1 = (float *)(send_info + 1);
//    data_ptr_1[0] = 234.4;
//    float * data_ptr_2 = (float *)(send_info + 5);
//    data_ptr_2[0] = 234.5;

//    float * data_ptr_3 = (float *)(send_info + 1);
//    qDebug()<<data_ptr_3[0]<<"   ";
//    float * data_ptr_4 = (float *)(send_info + 5);
//    qDebug()<<data_ptr_4[0];

//    qDebug()<<"size of send_info: "<<sizeof(send_info);
//    QByteArray c(QByteArray::fromRawData(send_info, sizeof(send_info)));
//    qDebug()<<"size of qbytearray: "<<sizeof(c);
//    char data_parrr[4];
//    for (int i = 1; i < 5; ++i) {
//         data_parrr[i-1] = c.at(i);
//    }

//    float * dddd = (float *)(data_parrr);
//    qDebug()<<dddd[0];

//    char data_parrrr[4];
//    for (int i = 1; i < 5; ++i) {
//         data_parrrr[i-1] = c.at(i+4);
//    }

//    float * ddddd = (float *)(data_parrrr);
//    qDebug()<<ddddd[0];


    w.show();
    return a.exec();
}
