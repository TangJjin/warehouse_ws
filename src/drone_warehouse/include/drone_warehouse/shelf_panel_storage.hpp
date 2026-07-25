#pragma once

#include <QString>
#include <QVector>

#include "drone_warehouse/models.hpp"

class ShelfPanelStorage
{
public:
    static QString defaultFilePath();
    static bool save(const QVector<ShelfPanelData> &shelves, QString *error_message = nullptr);
    static bool load(const QString &file_path,
                     int expected_slots_per_side,
                     QVector<ShelfPanelData> &shelves,
                     QString *error_message = nullptr);
    static bool load(int expected_slots_per_side,
                     QVector<ShelfPanelData> &shelves,
                     QString *error_message = nullptr);
};
