#ifndef VIEW_H
#define VIEW_H

#include <QGraphicsView>
#include <QObject>
#include "scene.h"

class View : public QGraphicsView
{
    Q_OBJECT
public:
    View(QWidget *parent);
    ~View();
    Scene *scene;
};

#endif // VIEW_H
