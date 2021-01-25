#include "OpenGLWidget.h"
#include <iostream>
#include "MazeWidget.h"
#include <gl\gl.h>
#include <gl\GLU.h>

OpenGLWidget::OpenGLWidget(QWidget *parent) : QGLWidget(parent)
{
	
	top_z = 1.5f;
	but_z = -1;

	QDir dir("Pic");
	if(dir.exists())
		pic_path = "Pic/";
	else
		pic_path = "../x64/Release/Pic/";
}
void OpenGLWidget::initializeGL()
{
	glClearColor(0,0,0,1);
	glEnable(GL_TEXTURE_2D);
	loadTexture2D(pic_path + "grass.png",grass_ID);
	loadTexture2D(pic_path + "sky.png",sky_ID);
}
void OpenGLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(MazeWidget::maze!=NULL)
	{
		//View 1
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glViewport(0 , 0 , MazeWidget::w/2 , MazeWidget::h);
		glOrtho (-0.1, MazeWidget::maze->max_xp +0.1, -0.1 , MazeWidget::maze->max_yp +0.1, 0 , 10);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		Mini_Map();

		//View 2
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glViewport(MazeWidget::w/2,0, MazeWidget::w/2, MazeWidget::h);
		/*gluPerspective 定義透視
		//視野大小, nearplane, farplane, distance
		//Note: You shouldn't use this function to get view matrix, otherwise you will get 0.
		*/
		//gluPerspective(MazeWidget::maze->viewer_fov, 1 , 0.01 , 200);

		/* gluLookAt
		//原本相機位置
		//看的方向
		//哪邊是上面
		//Note: You shouldn't use this function to get view matrix, otherwise you will get 0.
		*/
		float viewerPosX = MazeWidget::maze->viewer_posn[Maze::X];
		float viewerPosY = MazeWidget::maze->viewer_posn[Maze::Y];
		float viewerPosZ = MazeWidget::maze->viewer_posn[Maze::Z];

		/*gluLookAt(viewerPosX, viewerPosZ, viewerPosY,
			viewerPosX + cos(degree_change(MazeWidget::maze->viewer_dir)), viewerPosZ, viewerPosY + sin(degree_change(MazeWidget::maze->viewer_dir)),
			0.0, -1.0, 0.0);*/
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		Map_3D();
	}
}
void OpenGLWidget::resizeGL(int w,int h)
{
}

//Draw Left Part
void OpenGLWidget::Mini_Map()	
{					
	glBegin(GL_LINES);

		float viewerPosX = MazeWidget::maze->viewer_posn[Maze::X];
		float viewerPosY = MazeWidget::maze->viewer_posn[Maze::Y];
		float viewerPosZ = MazeWidget::maze->viewer_posn[Maze::Z];

		for(int i = 0 ; i < (int)MazeWidget::maze->num_edges; i++)
		{
			float edgeStartX = MazeWidget::maze->edges[i]->endpoints[Edge::START]->posn[Vertex::X];
			float edgeStartY = MazeWidget::maze->edges[i]->endpoints[Edge::START]->posn[Vertex::Y];
			float edgeEndX = MazeWidget::maze->edges[i]->endpoints[Edge::END]->posn[Vertex::X];
			float edgeEndY = MazeWidget::maze->edges[i]->endpoints[Edge::END]->posn[Vertex::Y];

			glColor3f(MazeWidget::maze->edges[i]->color[0] , MazeWidget::maze->edges[i]->color[1], MazeWidget::maze->edges[i]->color[2]);
			if(MazeWidget::maze->edges[i]->opaque)
			{
				glVertex2f(edgeStartX, edgeStartY);
				glVertex2f(edgeEndX, edgeEndY);
			}
		}

		//draw frustum
		float len = 0.1;
		glColor3f(1, 1, 1);
		glVertex2f(viewerPosX, viewerPosY);
		glVertex2f(viewerPosX + (MazeWidget::maze->max_xp) * len * cos(degree_change(MazeWidget::maze->viewer_dir - MazeWidget::maze->viewer_fov/2)) ,
			viewerPosY + (MazeWidget::maze->max_yp) * len * sin(degree_change(MazeWidget::maze->viewer_dir - MazeWidget::maze->viewer_fov/2)));

		glVertex2f(viewerPosX, viewerPosY);
		glVertex2f(viewerPosX + (MazeWidget::maze->max_xp) * len * cos(degree_change(MazeWidget::maze->viewer_dir + MazeWidget::maze->viewer_fov/2)) ,
			viewerPosY + (MazeWidget::maze->max_yp) * len *  sin(degree_change(MazeWidget::maze->viewer_dir + MazeWidget::maze->viewer_fov/2)));
	glEnd();
}


//**********************************************************************
//
// * Draws the first-person view of the maze.
//   THIS IS THE FUINCTION YOU SHOULD MODIFY.
//
//Note: You must not use any openGL build-in function to set model matrix, view matrix and projection matrix.
//		ex: gluPerspective, gluLookAt, glTraslatef, glRotatef... etc.
//		Otherwise, You will get 0 !
//======================================================================
void OpenGLWidget::Map_3D()
{
	glLoadIdentity();
	// 畫右邊區塊的所有東西
	
	float viewerPosX = MazeWidget::maze->viewer_posn[Maze::X];
	float viewerPosY = MazeWidget::maze->viewer_posn[Maze::Y];
	double maxWH = (MazeWidget::maze->max_xp >= MazeWidget::maze->max_yp) ? 
		MazeWidget::maze->max_xp : MazeWidget::maze->max_yp;

	double theta = MazeWidget::maze->viewer_fov / 2;
	double LeftRadian = degree_change(MazeWidget::maze->viewer_dir + theta);
	double RightRadian = degree_change(MazeWidget::maze->viewer_dir - theta);

	float LfrustumEndX = viewerPosX + (maxWH * 2) * cos(LeftRadian);
	float LfrustumEndY = viewerPosY + (maxWH * 2) * sin(LeftRadian);
	float RfrustumEndX = viewerPosX + (maxWH * 2) * cos(RightRadian);
	float RfrustumEndY = viewerPosY + (maxWH * 2) * sin(RightRadian);
	//create left and right frustum
	LineSeg leftFrustum(viewerPosX, viewerPosY, LfrustumEndX, LfrustumEndY);
	LineSeg rightFrustum(viewerPosX, viewerPosY, RfrustumEndX, RfrustumEndY);

	//find viewer in which cell
	Cell *viewerCell = NULL;
	for (int cell = 0; cell < MazeWidget::maze->num_cells; cell++)
	{
		Cell *currCell = MazeWidget::maze->cells[cell];

		//find current cell's left, right, Up, down
		double cellL = 2147483647, cellR = 0;
		double cellU = 0, cellD = 2147483647;
		for (int side = 0; side < 4; side++)
		{
			Edge *currSide = currCell->edges[side];
			double currSideStartX = currSide->endpoints[Edge::START]->posn[Vertex::X];
			double currSideStartY = currSide->endpoints[Edge::START]->posn[Vertex::Y];
			double currSideEndX = currSide->endpoints[Edge::END]->posn[Vertex::X];
			double currSideEndY = currSide->endpoints[Edge::END]->posn[Vertex::Y];

			double currSideMaxX = (currSideEndX >= currSideStartX) ? currSideEndX : currSideStartX;
			double currSideMinX = (currSideEndX <= currSideStartX) ? currSideEndX : currSideStartX;
			double currSideMaxY = (currSideEndY >= currSideStartY) ? currSideEndY : currSideStartY;
			double currSideMinY = (currSideEndY <= currSideStartY) ? currSideEndY : currSideStartY;

			if (currSideMaxX > cellR)
			{
				cellR = currSideMaxX;
			}
			if (currSideMinX < cellL)
			{
				cellL = currSideMinX;
			}
			if (currSideMaxY > cellU)
			{
				cellU = currSideMaxY;
			}
			if (currSideMinY < cellD)
			{
				cellD = currSideMinY;
			}
		}

		if (viewerPosX >= cellL && viewerPosX <= cellR && viewerPosY >= cellD && viewerPosY <= cellU)
		{
			viewerCell = currCell;
			break;
		}
	}

	//set footprint
	for (int cell = 0; cell < MazeWidget::maze->num_cells; cell++)
	{
		MazeWidget::maze->cells[cell]->footPrint = false;
	}
	
	/*若有興趣的話, 可以為地板或迷宮上貼圖, 此項目不影響評分*/
	//refrence: https://baike.baidu.com/item/glTexCoord2f
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, sky_ID);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex2f(-1, 0);
	glTexCoord2f(0, 1); glVertex2f(-1, 1);
	glTexCoord2f(1, 1); glVertex2f(1, 1);
	glTexCoord2f(1, 0); glVertex2f(1, 0);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, grass_ID);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex2f(-1, -1);
	glTexCoord2f(0, 1); glVertex2f(-1, 0);
	glTexCoord2f(1, 1); glVertex2f(1, 0);
	glTexCoord2f(1, 0); glVertex2f(1, -1);
	glEnd();

	glDisable(GL_TEXTURE_2D);
	
	//draw cell
	if (viewerCell == NULL)
	{
		//outside of maze
		throw new MazeException("Maze: Viewer not in maze\n");
	}
	else
	{
		//draw it
		Draw_Cell(viewerCell, leftFrustum, rightFrustum);
	}

}
void OpenGLWidget::Draw_Cell(Cell *C, LineSeg& Lfrustum, LineSeg& Rfrustum)
{
	C->footPrint = true;

	float viewerPosX = MazeWidget::maze->viewer_posn[Maze::X];
	float viewerPosY = MazeWidget::maze->viewer_posn[Maze::Y];

	for (int edge = 0; edge < 4; edge++)
	{
		Edge *currEdge = C->edges[edge];
		Vertex *edgeBegin = currEdge->endpoints[Edge::START];
		Vertex *edgeEnd = currEdge->endpoints[Edge::END];

		if (currEdge->Neighbor(C) != NULL && currEdge->Neighbor(C)->footPrint == true)
		{
			continue;
		}

		double LintersectX = 0, LintersectY = 0;
		double RintersectX = 0, RintersectY = 0;

		LineSeg currEdgeLine = LineSeg(edgeBegin->posn[Vertex::X], edgeBegin->posn[Vertex::Y],
			edgeEnd->posn[Vertex::X], edgeEnd->posn[Vertex::Y]);
		bool LIntersect = Intersect(Lfrustum, currEdgeLine, LintersectX, LintersectY);
		bool RIntersect = Intersect(Rfrustum, currEdgeLine, RintersectX, RintersectY);

		bool startPInFrustum = pointInFrustum(currEdgeLine.start[0], currEdgeLine.start[1], Lfrustum, Rfrustum);
		bool endPInFrustum = pointInFrustum(currEdgeLine.end[0], currEdgeLine.end[1], Lfrustum, Rfrustum);

		//clipping line
		bool draw = false;
		if (LIntersect && RIntersect)
		{
			//clip
			currEdgeLine.start[0] = LintersectX;
			currEdgeLine.start[1] = LintersectY;
			currEdgeLine.end[0] = RintersectX;
			currEdgeLine.end[1] = RintersectY;

			draw = true;
		}
		else if (!LIntersect && !RIntersect)
		{
			if (startPInFrustum && endPInFrustum)
			{
				draw = true;
			}
			else
			{
				draw = false;
			}
		}
		else if (LIntersect && !RIntersect)
		{
			if (startPInFrustum)
			{
				currEdgeLine.end[0] = LintersectX;
				currEdgeLine.end[1] = LintersectY;
			}
			else
			{
				currEdgeLine.start[0] = LintersectX;
				currEdgeLine.start[1] = LintersectY;
			}
			draw = true;
		}
		else if (!LIntersect && RIntersect)
		{
			if (startPInFrustum)
			{
				currEdgeLine.end[0] = RintersectX;
				currEdgeLine.end[1] = RintersectY;
			}
			else
			{
				currEdgeLine.start[0] = RintersectX;
				currEdgeLine.start[1] = RintersectY;
			}
			draw = true;
		}

		if (draw)
		{
			if (currEdge->opaque == true) // draw it
			{
				double halfFOV = MazeWidget::maze->viewer_fov / 2;

				//stable cot refrence: https://stackoverflow.com/questions/3738384/stable-cotangent
				if (halfFOV == 0)
				{
					halfFOV = 0.001;
				}
				double cot_halfFOV = cos(degree_change(halfFOV)) / sin(degree_change(halfFOV));
				double basicHeightDistance = cot_halfFOV * 1;

				//calculate distance
				double startPDistance = sqrt(pow(currEdgeLine.start[0] - viewerPosX, 2)
					+ pow(currEdgeLine.start[1] - viewerPosY, 2));
				double endPDistance = sqrt(pow(currEdgeLine.end[0] - viewerPosX, 2)
					+ pow(currEdgeLine.end[1] - viewerPosY, 2));
				if (startPDistance < basicHeightDistance)
				{
					startPDistance = basicHeightDistance;
				}
				if (endPDistance < basicHeightDistance)
				{
					endPDistance = basicHeightDistance;
				}

				//calculate height (positive Y position in screen coordinate)
				double startPHeight = basicHeightDistance / startPDistance;
				double endPHeight = basicHeightDistance / endPDistance;

				//calculate X position in screen coordinate

				//calculate original frustum
				double maxWH = (MazeWidget::maze->max_xp >= MazeWidget::maze->max_yp) ?
					MazeWidget::maze->max_xp : MazeWidget::maze->max_yp;
				double theta = MazeWidget::maze->viewer_fov / 2;
				double LRadian = degree_change(MazeWidget::maze->viewer_dir + theta);
				double RRadian = degree_change(MazeWidget::maze->viewer_dir - theta);
				float LfEndX = viewerPosX + (maxWH * 2) * cos(LRadian);
				float LfEndY = viewerPosY + (maxWH * 2) * sin(LRadian);
				float RfEndX = viewerPosX + (maxWH * 2) * cos(RRadian);
				float RfEndY = viewerPosY + (maxWH * 2) * sin(RRadian);
				//create left and right frustum
				LineSeg oriLFrustum(viewerPosX, viewerPosY, LfEndX, LfEndY);
				LineSeg oriRFrustum(viewerPosX, viewerPosY, RfEndX, RfEndY);

				double startPDeg = PointsToDegree(currEdgeLine.start[0], currEdgeLine.start[1], viewerPosX, viewerPosY);
				double endPDeg = PointsToDegree(currEdgeLine.end[0], currEdgeLine.end[1], viewerPosX, viewerPosY);
				double lDeg = PointsToDegree(oriLFrustum.end[0], oriLFrustum.end[1], viewerPosX, viewerPosY);
				double rDeg = PointsToDegree(oriRFrustum.end[0], oriRFrustum.end[1], viewerPosX, viewerPosY);

				double totalDeg = DegreeDifference(lDeg * 100 / 100, rDeg);
				double startToRDeg = DegreeDifference(startPDeg, rDeg);
				double endToRDeg = DegreeDifference(endPDeg, rDeg);
				if (totalDeg == 0)
				{
					totalDeg = 0.001;
				}

				double startP_X = ((1 - startToRDeg / totalDeg) * 2) - 1;
				double endP_X = ((1 - endToRDeg / totalDeg) * 2) - 1;
				
				//draw
				glColor3f(currEdge->color[0], currEdge->color[1], currEdge->color[2]);

				glBegin(GL_POLYGON);
				if (startP_X <= endP_X)
				{
					glVertex2f(startP_X, -startPHeight);
					glVertex2f(endP_X, -endPHeight);
					glVertex2f(endP_X, endPHeight);
					glVertex2f(startP_X, startPHeight);
				}
				else
				{
					glVertex2f(endP_X, -endPHeight);
					glVertex2f(startP_X, -startPHeight);
					glVertex2f(startP_X, startPHeight);
					glVertex2f(endP_X, endPHeight);
				}

				glEnd();
			}
			else
			{
				Cell *neighboor = currEdge->Neighbor(C);

				//decrease frustum
				double maxWH = (MazeWidget::maze->max_xp >= MazeWidget::maze->max_yp) ?
					MazeWidget::maze->max_xp : MazeWidget::maze->max_yp;

				double LeftRadian = 0, RightRadian = 0;

				if (currEdgeLine.start[0] < currEdgeLine.end[0])
				{
					LeftRadian = degree_change(PointsToDegree(currEdgeLine.start[0], currEdgeLine.start[1], 
						viewerPosX, viewerPosY));
					RightRadian = degree_change(PointsToDegree(currEdgeLine.end[0], currEdgeLine.end[1], 
						viewerPosX, viewerPosY));
				}
				else if (currEdgeLine.start[0] == currEdgeLine.end[0] && currEdgeLine.start[1] >= currEdgeLine.end[1])
				{
					LeftRadian = degree_change(PointsToDegree(currEdgeLine.start[0], currEdgeLine.start[1],
						viewerPosX, viewerPosY));
					RightRadian = degree_change(PointsToDegree(currEdgeLine.end[0], currEdgeLine.end[1],
						viewerPosX, viewerPosY));
				}
				else
				{
					RightRadian = degree_change(PointsToDegree(currEdgeLine.start[0], currEdgeLine.start[1],
						viewerPosX, viewerPosY));
					LeftRadian = degree_change(PointsToDegree(currEdgeLine.end[0], currEdgeLine.end[1],
						viewerPosX, viewerPosY));
				}

				float LfrustumEndX = viewerPosX + (maxWH * 2) * cos(LeftRadian);
				float LfrustumEndY = viewerPosY + (maxWH * 2) * sin(LeftRadian);
				float RfrustumEndX = viewerPosX + (maxWH * 2) * cos(RightRadian);
				float RfrustumEndY = viewerPosY + (maxWH * 2) * sin(RightRadian);
				//create left and right frustum
				LineSeg leftFrustum(viewerPosX, viewerPosY, LfrustumEndX, LfrustumEndY);
				LineSeg rightFrustum(viewerPosX, viewerPosY, RfrustumEndX, RfrustumEndY);

				//printf("L: %lf %lf %lf %lf %lf, ", viewerPosX, viewerPosY, LfrustumEndX, LfrustumEndY, LeftRadian);
				//printf("R: %lf %lf %lf %lf %lf\n", viewerPosX, viewerPosY, RfrustumEndX, RfrustumEndY, RightRadian);

				//recursive call
				Draw_Cell(neighboor, leftFrustum, rightFrustum);
			}
		}
	}
}
double OpenGLWidget::PointsToDegree(double x1, double y1, double xref, double yref)
{
	double dx = x1 - xref;
	double dy = y1 - yref;
	double tan = (x1 == xref) ? ((y1 >= 0) ? 2147483647 : -2147483648) : dy / dx;
	double theta = atan(tan) * 180 / M_PI;

	if (x1 < xref)
	{
		if (theta > 0)
		{
			theta = theta - 180;
		}
		else
		{
			theta = theta + 180;
		}
	}
	return (int)((theta + 0.005) * 100) / 100.0;
}
double OpenGLWidget::DegreeDifference(double lDeg, double rDeg)
{	
	double degDiff;
	if (lDeg < rDeg)
	{
		degDiff = (lDeg + 360) - rDeg;
	}
	else
	{
		degDiff = lDeg - rDeg;
	}
	return degDiff;
}
bool OpenGLWidget::Intersect(LineSeg& frustum, LineSeg& edge, double& intersectX, double& intersectY)
{
	double frag = edge.Cross_Param(frustum);

	if (frag > 1 || frag < 0)
	{
		return false;
	}
	else
	{
		//double check
		char frustumStartSide = pointSide(edge, frustum.start[0], frustum.start[1]);
		char frustumEndSide = pointSide(edge, frustum.end[0], frustum.end[1]);

		if (frustumEndSide == frustumStartSide)
		{
			return false;
		}
		
		//calculate intersect point
		intersectX = (1 - frag) * edge.start[0] + frag * edge.end[0];
		intersectY = (1 - frag) * edge.start[1] + frag * edge.end[1];

		return true;
	}
}
bool OpenGLWidget::pointInFrustum(double x, double y, LineSeg& Lfrustum, LineSeg& Rfrustum)
{
	float viewerPosX = MazeWidget::maze->viewer_posn[Maze::X];
	float viewerPosY = MazeWidget::maze->viewer_posn[Maze::Y];
	double lDeg = PointsToDegree(Lfrustum.end[0], Lfrustum.end[1], viewerPosX, viewerPosY);
	double rDeg = PointsToDegree(Rfrustum.end[0], Rfrustum.end[1], viewerPosX, viewerPosY);
	double p = PointsToDegree(x, y, viewerPosX, viewerPosY);
	
	if (lDeg >= rDeg)
	{
		if (p >= rDeg && p <= lDeg)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (p >= lDeg && p <= rDeg)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
}
char OpenGLWidget::pointSide(LineSeg& line, double x, double y)
{
	/*double det = endpoints[START]->posn[Vertex::X] *
		(endpoints[END]->posn[Vertex::Y] - y) -
		endpoints[START]->posn[Vertex::Y] *
		(endpoints[END]->posn[Vertex::X] - x) +
		endpoints[END]->posn[Vertex::X] * y -
		endpoints[END]->posn[Vertex::Y] * x;*/
	double det = line.start[0] * (line.end[1] - y) - line.start[1] * (line.end[0] - x) +
		line.end[0] * y - line.end[1] * x;

	if (det == 0.0)
		return Edge::ON;
	else if (det > 0.0)
		return Edge::LEFT;
	else
		return Edge::RIGHT;
}

void OpenGLWidget::loadTexture2D(QString str,GLuint &textureID)
{
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	
	QImage img(str);
	QImage opengl_grass = QGLWidget::convertToGLFormat(img);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, opengl_grass.width(), opengl_grass.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, opengl_grass.bits());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glDisable(GL_TEXTURE_2D);
}
float OpenGLWidget::degree_change(float num)
{
	return num /180.0f * 3.14159f;
}