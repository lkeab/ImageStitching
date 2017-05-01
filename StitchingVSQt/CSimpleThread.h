#ifndef CSIMPLETHREAD_H
#define CSIMPLETHREAD_H
#include <QThread>

class CSimpleThread : public QThread
{
	Q_OBJECT
public:
	CSimpleThread();
	void run();
};

#endif // CSIMPLETHREAD_H