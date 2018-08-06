/*!
 * @file
 * */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <QDialog>
#include <QMainWindow>

namespace Ui {
class Settings;
}

/*!
 * @brief Settings is the settings class for the settings window
 */
class Settings : public QDialog {
    Q_OBJECT

public:
    /*!
     * @brief This is the default constructor
     */
    Settings();
    /*!
     * @brief The constructor for creating the settings window
     * @param[in] *parent - The object of the parent Window
     * @param[in] IP - The IP which is currently stored in the settings file and should be displayed
     * @param[in] Port - The Port which is currently stored in the settings file and should be displayed
     * @param[in] Dev - The device name of the gamepad which is currently saved in the settings file and should be set as active
     */
    Settings(QMainWindow *parent, QString IP, QString Port, QString Dev);
    /*!
     * @brief This is the destructor
     */
    ~Settings();
    /*!
     * @brief open() this function opens the settings window
     */
    void open();

    /// The settings user interface for setting data in the gui
    Ui::Settings *settings_ui;
};

#endif // SETTINGS_H
