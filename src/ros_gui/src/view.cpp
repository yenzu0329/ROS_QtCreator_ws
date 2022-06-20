#include "view.h"

View::View(QWidget *parent): QGraphicsView(parent)
{
    scene = new Scene(0, 0, 936, 696);
    setScene(scene);
    setMouseTracking(true);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    scale(0.7, 0.7);
}

View::~View()
{
    delete scene;
}
