#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDesktopWidget>
#include <QCloseEvent>

#include "communication/ClientSocket.h"
#include "communication/SocketException.h"
#include <QGraphicsScene>
#include <QGraphicsView>

/**
 * /CLASS ClientSocket
 *
 * creates a MainWindow object
 */
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private:
    Ui::MainWindow *ui;
    ClientSocket *client;

    bool connected, closeFlag, streamMapFlag;
    QGraphicsScene * scene;
    QVector <QPointF> points;


private slots:

    void initSlots();
    void closeWindow();

    // Qt events
    void closeEvent(QCloseEvent *event);

    // connect/disconnect to server on Alf
    void conServer();
    void disConServer();

    // SlamMap
    void startStreamMap();
    void stopStreamMap();
    void saveMap();
    void resetMap();


    void run();
    void createTxtMapFile(std::string fileName, std::vector<int>);
    void savePGM(std::vector<int> v);

    void fill(std::vector<int>);
};

#endif // MAINWINDOW_H
