#include <QtGui/QApplication>
#include "improvedTrICP.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    improvedTrICP foo;
    foo.show();
    return app.exec();
}
