#pragma once
#include <QGLWidget>
#include <QString>
#include <QDir>
#include "LineSeg.h"
class OpenGLWidget :public QGLWidget
{
	Q_OBJECT
public:
	explicit OpenGLWidget(QWidget *parent = 0);

	void initializeGL();
	void paintGL();
	void resizeGL(int ,int );

	//Maze Setting
	void Mini_Map();
	void Map_3D();
	void Draw_Cell(Cell *C, LineSeg& Lfrustum, LineSeg& Rfrustum);
	double PointsToDegree(double x1, double y1, double xref, double yref);
	double DegreeDifference(double lDeg, double rDeg);
	bool Intersect(LineSeg& frustum, LineSeg& edge, double& intersectX, double& intersectY);
	bool pointInFrustum(double x, double y, LineSeg& Lfrustum, LineSeg& Rfrustum);
	char pointSide(LineSeg& line, double x, double y);
	void loadTexture2D(QString, GLuint &);
	float degree_change(float );
private:
	GLuint grass_ID;
	GLuint sky_ID;
	QString pic_path;

	float top_z;
	float but_z;
};

