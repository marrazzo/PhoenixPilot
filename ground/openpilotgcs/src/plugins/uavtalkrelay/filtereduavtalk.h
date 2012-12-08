/**
 ******************************************************************************
 * @file       filtereduavtalk.h
 * @author     The PhoenixPilot Team, http://github.com/PhoenixPilot
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup UAVTalk relay plugin
 * @{
 *
 * @brief Relays UAVTalk data trough UDP to another GCS
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef FILTEREDUAVTALK_H
#define FILTEREDUAVTALK_H

#include "../uavtalk/uavtalk.h"
#include "uavtalkrelayplugin.h"
#include <QHash>

class FilteredUavTalk:public UAVTalk
{
    Q_OBJECT
public:
    FilteredUavTalk(QIODevice* iodev, UAVObjectManager* objMngr,QHash<quint32,UavTalkRelay::accessType> rules,UavTalkRelay::accessType defaultRule);
    bool receiveObject(quint8 type, quint32 objId, quint16 instId, quint8* data, qint32 length);
private:
    QHash<quint32,UavTalkRelay::accessType> m_rules;
    UavTalkRelay::accessType m_defaultRule;
public slots:
    void sendObjectSlot(UAVObject *obj);
};

#endif // FILTEREDUAVTALK_H
