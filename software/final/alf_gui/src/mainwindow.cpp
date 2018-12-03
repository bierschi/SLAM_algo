
#include <iostream>
#include <thread>
#include <iomanip>
#include <QDebug>
#include <QtWidgets/QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"



/**
 * Constructor for a MainWindow instance
 *
 * USAGE:
 *      MainWindow w;
 *
 * @param parent: QWidget*
 */
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow),
                                          client(NULL),
                                          connected(false),
                                          closeFlag(true),
                                          streamMapFlag(false)
{
    // setup GUI
    ui->setupUi(this);
    this->setWindowTitle("ALF");
    //this->setGeometry(QStyle::alignedRect(Qt::LeftToRight, Qt::AlignCenter, this->size(), qApp->desktop()->availableGeometry()));
    QDesktopWidget dw;
    int x = dw.width() * 0.6;
    int y = dw.height() * 0.8;
    this->setFixedSize(x, y);
    // init slots
    initSlots();

    ui->disconnect_pb->setEnabled(false);

    //only for testing purposes
    //ui->host_le->setText("192.168.178.39");
    ui->host_le->setText("localhost");
    ui->port_le->setText("2501");


    scene = new QGraphicsScene(this);
    scene->setSceneRect(50, 50, 350, 350);
    ui->slam_gv->setScene(scene);
    //fill();


}

void MainWindow::fill(std::vector<int> v) {
    QPainter painter(this);

    QVector <QPointF> points;
    for(unsigned int y = 0; y < 400; y++) {
        for(unsigned int x = 0; x < 400; x++) {

            unsigned int i = x + (400 -y -1) * 400;

            if (v[i] == 254) {

                points.append(QPointF(5, 5));

            } else if (v[i] == 0) {

                points.append(QPointF(15, 15));

            } else {

                points.append(QPointF(25, 25));

            }
        }
    }


}

/**
 * initialize all slots
 */
void MainWindow::initSlots() {

    connect(ui->exit_pb, SIGNAL(clicked()), this, SLOT(closeWindow()));
    connect(ui->connect_pb, SIGNAL(clicked()), this, SLOT(conServer()));
    connect(ui->disconnect_pb, SIGNAL(clicked()), this, SLOT(disConServer()));

    connect(ui->save_map_pb, SIGNAL(clicked()), this, SLOT(saveMap()));
    connect(ui->reset_map_pb, SIGNAL(clicked()), this, SLOT(resetMap()));
    connect(ui->start_stream_map_pb, SIGNAL(clicked()), this, SLOT(startStreamMap()));
    connect(ui->stop_stream_map_pb, SIGNAL(clicked()), this, SLOT(stopStreamMap()));
}


/**
 * Destructor in MainWindow
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/**
 * close the main window
 */
void MainWindow::closeWindow() {

    QMessageBox::StandardButton resBtn = QMessageBox::question(this, "ALF", tr("Are you sure to quit?"));

    if (resBtn != QMessageBox::Yes) {

    } else {

        exit(0);

    }
}


/**
 * close the main window
 *
 * @param event: QCloseEvent*
 */
void MainWindow::closeEvent(QCloseEvent *event) {

    QMessageBox::StandardButton resBtn = QMessageBox::question(this, "ALF", tr("Are you sure to quit?"));

    if (resBtn != QMessageBox::Yes) {

        event->ignore();

    } else {
        closeFlag = false;
        disConServer();
        event->accept();

    }
}

/**
 * connect to the server on ALF
 */
void MainWindow::conServer() {

    std::string host = ui->host_le->text().toStdString();
    unsigned int port = (unsigned) ui->port_le->text().toInt();
    std::cout << "Set up connection ..." << std::endl;

    try {

        client = new ClientSocket(host, port);

        QMessageBox::information(this, "Information", "Connection established!");

        connected = true;

        ui->connect_pb->setEnabled(false);
        ui->disconnect_pb->setEnabled(true);

    } catch ( SocketException& e ) {

        std::cout << "SocketException caught with error message: " << e.description() << std::endl;
        QMessageBox::warning(this, "Error", "Connection failed!");

        connected = false;
    }
}

/**
 * disconnect from the server on ALF
 */
void MainWindow::disConServer() {

    if (connected) {

        if (closeFlag)
            QMessageBox::information(this, "Information", "Connection closed!");

        ui->disconnect_pb->setEnabled(false);
        ui->connect_pb->setEnabled(true);

        connected = false;
        client->closeConnection();

    }
}

void MainWindow::startStreamMap() {
    Commands start_stream_map = START_STREAM_MAP;

    if (connected) {

        client->sending(start_stream_map);
        streamMapFlag = true;
        std::thread r(&MainWindow::run, this);
        r.detach();

    }
}

void MainWindow::stopStreamMap() {
    Commands stop_stream_map = STOP_STREAM_MAP;

    if (connected) {
        streamMapFlag = false;
        client->sending(stop_stream_map);
    }

}

/**
 * saves a slam map as a pgm file
 */
void MainWindow::saveMap() {
    Commands save_map = SAVE_MAP;
    if (connected)
        client->sending(save_map);
}

/**
 * resets the current slam map
 */
void MainWindow::resetMap() {
    Commands reset_map = RESET_MAP;
    if (connected)
        client->sending(reset_map);
}

void MainWindow::run() {

    std::vector<int> v;

    while (streamMapFlag) {

        if (connected) {
            client->receiving(v);
            savePGM(v);
            //fill(v);
            v.clear();

        }

        sleep(5);

    }
}

void MainWindow::savePGM(std::vector<int>& v) {
    //std::string mapCounterStr = std::to_string(mapCounter_);
    //std::string filename = "map" + mapCounterStr +  ".pgm";
    //FILE* out = fopen(filename.c_str(), "w");
    FILE* out = fopen("gui.pgm", "w");
    if (!out) {

        return;
    }

    fprintf(out, "P2\n%d %d 255\n",
            400,
            400);
    for(unsigned int y = 0; y < 400; y++) {
        for(unsigned int x = 0; x < 400; x++) {

            unsigned int i = x + (400 -y -1) * 400;

            if (v[i] == 254) {

                //fputc(254, out);
                fprintf(out, "%d ",254);

            } else if (v[i] == 0) {

                //fputc(000, out);
                fprintf(out, "%d ",000);

            } else {

                //fputc(205, out);
                fprintf(out, "%d ",205);

            }
        }
        fprintf(out, "\n");
    }

    fclose(out);
    mapCounter_++;
}


void MainWindow::createTxtMapFile(std::string fileName, std::vector<int> mapData) {

    FILE* out = fopen(fileName.c_str(), "w");
    for(int s = 0; s < mapData.size(); s++) {
        fprintf(out, "%d ", mapData[s]);

        if (s && s%400 == 0) {

            fprintf(out, "\n");

        }
    }
    fclose(out);
}