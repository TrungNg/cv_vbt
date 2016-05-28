// Detections.cpp : implementation file
//

#include "stdafx.h"
//#define _USE_MATH_DEFINES
#include <cmath>
#include <time.h>
#include "Detections.h"
#include "Shapes.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#define M_PI					3.14159265358979323846
#define	cos30					0.866025f
#define sin30					0.5f

#define	LEFTBOUND_0				23.0f*M_PI/12
#define	LEFTBOUND_1				M_PI/12
#define	LEFTBOUND_2				M_PI/4
#define	LEFTBOUND_3				5.0f*M_PI/12
#define	LEFTBOUND_4				7.0f*M_PI/12
#define	LEFTBOUND_5				3.0f*M_PI/4
#define	LEFTBOUND_6				11.0f*M_PI/12
#define	LEFTBOUND_7				13.0f*M_PI/12
#define	LEFTBOUND_8				5.0f*M_PI/4
#define	LEFTBOUND_9				17.0f*M_PI/12
#define	LEFTBOUND_10			19.0f*M_PI/12
#define	LEFTBOUND_11			7.0f*M_PI/4
#define	BIN_0					0
#define	BIN_1					M_PI/6
#define	BIN_2					M_PI/3
#define	BIN_3					M_PI/2
#define	BIN_4					2.0f*M_PI/3
#define	BIN_5					5.0f*M_PI/6
#define	BIN_6					M_PI
#define	BIN_7					7.0f*M_PI/6
#define	BIN_8					8.0f*M_PI/3
#define	BIN_9					3.0f*M_PI/2
#define	BIN_10					5.0f*M_PI/3
#define	BIN_11					11.0f*M_PI/6
#define VERTEX_EPSILON			2
#define VERTEX_THRESHOLD		1000		// ~ (30 * 20) / 3
#define	INCENTER_THRESHOLD		3000

#define R_max		160
#define R_max_2		80
#define BI_min		20
#define BI_max		140
#define A_CENTER_max	96
#define A_CENTER_max_x	84


/********************************************************************************/
std::vector<Triangle*> VBT_detect_uptriangle(Mat grad_x, Mat grad_y, Mat grad);

/*
 */
void colorTransform(Mat input, Mat &output, cl_prior_t type)
{
	float kg, kr, kb;
	switch (type){
	case NONE:
		kr = 0.299;
		kg = 0.587;
		kb = 0.114;
		break;
	case RED:
		kr = 0;
		kg = 0.837;
		kb = 0.163;
		break;
	case BLUE:
		kr = 0.337;
		kg = 0.663;
		kb = 0;
	}
	bgr_t *bgrin = (bgr_t *)input.data;
	uint8_t *out = (uint8_t *)output.data;
	for (register int i = 0; i < input.rows; i++)
	for (register int j = 0; j < input.cols; j++, bgrin++, out++){
		*out = kr*bgrin->r + kg*bgrin->g + kb*bgrin->b;
	}
}

/*
 */
void computeGradients(Mat input, Mat &gx, Mat &gy, Mat &mag, double threshold)
{
	Mat workImg = Mat(input);
	workImg = input.clone();


	Mat magX = Mat(input.rows, input.cols, CV_32F);
	Mat magY = Mat(input.rows, input.cols, CV_32F);
	Sobel(workImg, magX, CV_32F, 1, 0, 5);
	Sobel(workImg, magY, CV_32F, 0, 1, 5);

	for (int i = 0; i < input.rows; i++)
	for (int j = 0; j < input.cols; j++) {
		float mx = magX.at<float>(i, j);
		float my = magY.at<float>(i, j);
		float mg = (float)::sqrt(mx*mx + my*my);
		if (mg < threshold) {
			mag.at<float>(i, j) = gx.at<float>(i, j) = gy.at<float>(i, j) = 0.0f;
		}
		else {
			mx /= mg; my /= mg;
			mag.at<float>(i, j) = mg;
			gx.at<float>(i, j) = mx;
			gy.at<float>(i, j) = my;
		}
	}
}

/*
*/
void Detector::preproc(double threshold)
{
	V = Mat::zeros(r, c, CV_32F);
	gx = Mat(r, c, CV_32FC1);
	gy = Mat(r, c, CV_32FC1);
	mag = Mat(r, c, CV_32FC1);
	::computeGradients(img, gx, gy, mag, threshold);
	//::normalizeFloatImage(mag);
}

/*
/*
 */
std::vector<Triangle*> Detector::findShape(int nsides, int maxSize, int minSize)
{
	//if (nsides == 3) return findCircles(rs);
	//else return findPolygons(nsides, rs);
	std::vector<Triangle*> ret;
	switch (nsides){
	case 3:
		return VBT_detect_uptriangle(gx, gy, mag);
	}
	return ret;
}


int top_hat(int input, int thresh)
{
	return ((input > thresh) ? 0 : 1);
}


// accumulate "weight" to line segment between point a and point b
void vote(const Point &a, const Point &b, float weight, Mat &vote_img)
{
	LineIterator it_votes(vote_img, a, b, 8);

	for (int i = 0; i < it_votes.count; i++, ++it_votes) {
		float *vote_ptr = (float *)(*it_votes);
		(*vote_ptr) += weight;
	}
}

/* detect up pointed triangle sign : /_\
/* 
 */
std::vector<Triangle*> VBT_detect_uptriangle(Mat grad_x, Mat grad_y, Mat grad)
{
	clock_t t0, t1;
	t0 = clock();
	int w = grad_x.cols, h = grad_x.rows;
	std::vector<Triangle*> ret;
	Mat tl = Mat::zeros(h, w, CV_32FC4);
	Mat tr = Mat::zeros(h, w, CV_32FC4);
	Mat bt = Mat::zeros(h, w, CV_32FC4);

	float *grx = (float *)grad_x.data, *gry = (float *)grad_y.data, *gr = (float *)grad.data;
	float *btgr = (float *)bt.data, *tlgr = (float *)tl.data, *trgr = (float *)tr.data;
	
	// get orientation map
	float theta, edge_tan;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, grx++, gry++, gr++, trgr += 4, tlgr += 4, btgr += 4){
			if (!(*gr))	continue;
			theta = atan2(*gry, *grx);
			if (theta < 0)
				theta += M_PI + M_PI;
			if (*gry){
				edge_tan = -(*grx) / (*gry);
			}
			else if (*grx > 0){
				edge_tan = INT_MAX;
			}
			else {
				edge_tan = INT_MIN;
			}
			if ((theta <= LEFTBOUND_10) && (theta > LEFTBOUND_9))
			{
				*btgr = *gr; *(btgr + 1) = theta; *(btgr + 2) = edge_tan; continue;
			}
			if ((theta <= LEFTBOUND_2) && (theta > LEFTBOUND_1))
			{
				*tlgr = *gr; *(tlgr + 1) = theta; *(tlgr + 2) = edge_tan; continue;
			}
			if ((theta <= LEFTBOUND_6) && (theta > LEFTBOUND_5))
			{
				*trgr = *gr; *(trgr + 1) = theta; *(trgr + 2) = edge_tan; continue;
			}
		}
	}

	// Vertex accumulator map
	Mat lv = Mat::zeros(h, w, CV_32FC1);
	Mat rv = Mat::zeros(h, w, CV_32FC1);
	Mat tv = Mat::zeros(h, w, CV_32FC1);
	btgr = (float *)bt.data;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, btgr+=4){
			if (*btgr){
				//float ri;

				// vote left vertex
				int i1_min = (i > BI_max) ? (i - BI_max) : 0, j1_min = (j > R_max) ? (j - R_max) : 0, \
					i1_max = (i + 20 < h) ? (i + 20) : h, j1_max = (j + R_max_2 < w) ? (j + R_max_2) : w;
				float *tlgr_mag_off = (float *)tl.data + 4 * (i1_min*w + j1_min);

				for (int i1 = i1_min; i1 < i1_max; i1++, tlgr_mag_off += 4 * w){
					float *tl_mg = tlgr_mag_off;
					for (int j1 = j1_min; j1 < j1_max; j1++, tl_mg+=4){
						if (!(*tl_mg)) continue;
						if (!top_hat(sqrt((i - i1)*(i - i1) + (j - j1)*(j - j1)), R_max)) continue;
						if ((i <= i1 + (*(tl_mg + 2)) * (j - j1) + VERTEX_EPSILON) || \
							(i1 >= i + (*(btgr + 2)) * (j1 - j) - VERTEX_EPSILON))
							continue;
						/*if (i > h / 2 && j > w / 2)
							ri = 1;*/
						float ai = *(btgr + 2), aj = *(tl_mg + 2);
						float xA = (i1 - aj*j1 - i + ai*j) / (ai - aj);
						float yA = ai*(xA - j) + i;
						if ((yA < 20) || (xA > w - 25) || (xA <= 0) || (yA >= h-1)) continue;
						
						/*if (*(btgr + 3) == 0)
							(*(btgr + 3)) = ri = log10(1 + (*(btgr)));*/
						//if (*(tl_mg + 3) != 0)
						//	lv.at<float>(yA, xA) ++ /* Dij * */ /*ri * (*(tl_mg + 3))*/;
						//else{
						//	*(tl_mg + 3) = log10(1 + (*(tl_mg)));
						lv.at<float>(yA + 0.5, xA + 0.5)++ /* Dij * */ /*ri * (*(tl_mg + 3))*/;
						//}
					}
				}

				// vote right vertex
				i1_min = (i > BI_max) ? (i - BI_max) : 0; j1_min = (j > R_max_2) ? (j - R_max_2) : 0;
				i1_max = (i + 20 < h) ? (i + 20) : h; j1_max = (j + R_max < w) ? (j + R_max) : w;
				float *trgr_mag_off = (float *)tr.data + 4 * (i1_min*w + j1_min);
				
				for (int i1 = i1_min; i1 < i1_max; i1++, trgr_mag_off += 4 * w){
					float *tr_mg = trgr_mag_off;
					for (int j1 = j1_min; j1 < j1_max; j1++, tr_mg += 4){
						if (!(*tr_mg)) continue;
						if (!top_hat(sqrt((i - i1)*(i - i1) + (j - j1)*(j - j1)), R_max)) continue;
						if ((i1 >= i + (*(btgr + 2)) * (j1 - j) - VERTEX_EPSILON) || \
							(i <= i1 + (*(tr_mg + 2)) * (j - j1) + VERTEX_EPSILON))
							continue;

						float ai = *(btgr + 2), aj = *(tr_mg + 2);
						float xA = (i1 - aj*j1 - i + ai*j) / (ai - aj);
						float yA = ai*(xA - j) + i;
						if ((yA < 20) || (xA < 25) || (xA >= w-1) || (yA >= h-1)) continue;
						
						//if (*(btgr + 3) == 0)
						//	(*(btgr + 3)) = ri = log10(1 + (*(btgr)));
						//if (*(tr_mg + 3) != 0)
						//	rv.at<float>(yA, xA) ++ /* Dij * */ /*ri * (*(tr_mg + 3))*/;
						//else{
						//	*(tr_mg + 3) = log10(1 + (*(tr_mg)));
						rv.at<float>(yA + 0.5, xA + 0.5)++ /* Dij * */ /*ri * (*(tr_mg + 3))*/;
						//}
					}
				}
			}
		}
	}

	tlgr = (float *)tl.data;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, tlgr += 4){
			if (*tlgr){
				//float ri = *(tlgr + 3);

				// vote top vertex
				int i1_min = (i > BI_max) ? (i - BI_max) : 0, \
					i1_max = (i + BI_max < h) ? (i + BI_max) : h, j1_max = (j + R_max < w) ? (j + R_max) : w;
				float *trgr_mag_off = (float *)tr.data + 4*(i1_min*w + j);
				for (int i1 = i1_min; i1 < i1_max; i1++, trgr_mag_off += 4 * w){
					float *tr_mg = trgr_mag_off;
					for (int j1 = j; j1 < j1_max; j1++, tr_mg += 4){
						if (!(*tr_mg)) continue;
						if (!top_hat(sqrt((i - i1)*(i - i1) + (j - j1)*(j - j1)), R_max)) continue;
						if ((i1 <= i + (*(tlgr + 2)) * (j1 - j) + VERTEX_EPSILON) || \
							(i <= i1 + (*(tr_mg + 2)) * (j - j1) + VERTEX_EPSILON))
							continue;

						float ai = *(tlgr + 2), aj = *(tr_mg + 2);
						float xA = (i1 - aj*j1 - i + ai*j) / (ai - aj);
						float yA = ai*(xA - j) + i;
						/*if (xA < 0 || xA >= w || yA < 0 || yA >= h)	continue;*/
						if ((yA > h - 20) || (xA <= 12) || (xA >= w - 13) || (yA <= 0)) continue;

						//if (*(tlgr + 3) == 0)
						//	(*(tlgr + 3)) = ri = log10(1 + (*tlgr));
						//if (*(tr_mg + 3) != 0)
						//	tv.at<float>(yA, xA) ++ /* Dij * */ /*ri * (*(tr_mg + 3))*/;
						//else{
						//	*(tr_mg + 3) = log10(1 + (*(tr_mg)));
							tv.at<float>(yA + 0.5, xA + 0.5) ++ /* Dij * */ /*ri * (*(tr_mg + 3))*/;
						//}
					}
				}
			}
		}
	}
	/***********************Testing*****************/
	/*double maxim;
	minMaxLoc(tv, NULL, &maxim);
	tv *= 1.0f / maxim;
	minMaxLoc(rv, NULL, &maxim);
	rv *= 1.0f / maxim;
	minMaxLoc(lv, NULL, &maxim);
	lv *= 1.0f / maxim;
	imshow("tv", tv);
	imshow("lv", lv);
	imshow("rv", rv);
	imshow("vertices", tv + rv + lv);
	waitKey(0);*/

	/************************************************/
	/*boxFilter(tv, tv, -1, Size(3, 3), Point(-1, -1), false);
	boxFilter(rv, rv, -1, Size(3, 3), Point(-1, -1), false);
	boxFilter(lv, lv, -1, Size(3, 3), Point(-1, -1), false);*/
	/*int tx, ty, lx, ly, rx, ry;*/
	Mat lv1(h, w, CV_32FC1);
	Mat rv1(h, w, CV_32FC1);
	Mat tv1(h, w, CV_32FC1);
	// find local maxima
	float *topv = (float *)tv.data + 1 + w, *rightv = (float *)rv.data + 1 + w, *leftv = (float *)lv.data + 1 + w, \
		*topv1 = (float *)tv1.data + 1 + w, *rightv1 = (float *)rv1.data + 1 + w, *leftv1 = (float *)lv1.data + 1 + w;
	/*float maxT = 0, maxL = 0, maxR = 0; int xm = 0, ym  = 0;*/
	for (int i = 1; i < h-1; i++){
		float M;
		for (int j = 1; j < w-1; j++, topv++, rightv++, leftv++, topv1++, rightv1++, leftv1++){
			M = *topv;
			/*if ( (M <= VERTEX_THRESHOLD) || (M < *(topv + 1)) || (M <= *(topv - 1)) || (M <= *(topv - w)) || (M < *(topv + w)) \
				|| (M < *(topv + w - 1)) || (M <= *(topv - w + 1)) || (M < *(topv + w + 1)) || (M <= *(topv - w - 1)) )
				*topv1 = 0;
			else {
				*topv1 = M;
				}*/
			if (M <= VERTEX_THRESHOLD) M = 0;
			else{
				for (int i1 = max(i - 10, 0); i1 < min(i + 10, h); i1++){
					for (int j1 = max(j - 10, 0); j1 < min(j + 10, w); j1++){
						if (M < tv.at<float>(i1, j1)){
							M = 0;
							i1 = i + 10;
							j1 = j + 10;
						}
					}
				}
			}
			*topv1 = M;
			/*if (M)
			{
				tx = j;
				ty = i;
			}*/

			M = *rightv;
			/*if ((M <= VERTEX_THRESHOLD) || (M < *(rightv + 1)) || (M <= *(rightv - 1)) || (M <= *(rightv - w)) || (M < *(rightv + w)) \
				|| (M < *(rightv + w - 1)) || (M <= *(rightv - w + 1)) || (M < *(rightv + w + 1)) || (M <= *(rightv - w - 1)))
				*rightv1 = 0;
			else{
				*rightv1 = M;
				}*/
			if (M <= VERTEX_THRESHOLD) M = 0;
			else{
				for (int i1 = max(i - 10, 0); i1 < min(i + 10, h); i1++){
					for (int j1 = max(j - 10, 0); j1 < min(j + 10, w); j1++){
						if (M < rv.at<float>(i1, j1)){
							M = 0;
							i1 = i + 10;
							j1 = j + 10;
						}
					}
				}
			}
			*rightv1 = M;
			/*if (M)
			{
				rx = j;
				ry = i;
			}*/


			M = *leftv;
			/*if ( (M  <= VERTEX_THRESHOLD) || (M < *(leftv + 1)) || (M <= *(leftv - 1)) || (M <= *(leftv - w)) || (M < *(leftv + w)) \
				|| (M < *(leftv + w - 1)) || (M <= *(leftv - w + 1)) || (M < *(leftv + w + 1)) || (M <= *(leftv - w - 1)))
				*leftv1 = 0;
			else{
				*leftv1 = M;
				}*/
			if (M <= VERTEX_THRESHOLD) M = 0;
			else{
				for (int i1 = max(i - 10, 0); i1 < min(i + 10, h); i1++){
					for (int j1 = max(j - 10, 0); j1 < min(j + 10, w); j1++){
						if (M < lv.at<float>(i1, j1)){
							M = 0;
							i1 = i + 10;
							j1 = j + 10;
						}
					}
				}
			}
			*leftv1 = M;
			/*if (M)
			{
				lx = j;
				ly = i;
			}*/
		}
		topv += 2, rightv += 2, leftv += 2, topv1 += 2, rightv1 += 2, leftv1 += 2;
	}

	/************************** Testing ************************/
	/*tv1 = tv1 + lv1 + rv1;
	Mat org = imread("D:\\Working\\Box Sync\\StopSignDetection\\data\\test_upright_triangle_2.bmp", CV_LOAD_IMAGE_COLOR);*/
	/*double maxim;
	minMaxLoc(tv1, NULL, &maxim);
	tv1 *= 1.0f / maxim;
	minMaxLoc(rv1, NULL, &maxim);
	rv1 *= 1.0f / maxim;
	minMaxLoc(lv1, NULL, &maxim);
	lv1 *= 1.0f / maxim;
	imshow("tv1", tv1);
	imshow("lv1", lv1);
	imshow("rv1", rv1);*/
	/*line(org, Point(tx, ty), Point(tx, ty), Scalar(0, 0, 255), 3);
	line(org, Point(rx, ry), Point(rx, ry), Scalar(0, 0, 255), 3);
	line(org, Point(lx, ly), Point(lx, ly), Scalar(0, 0, 255), 3);*/
	/*imshow("original image", org);*/
	/*waitKey(0);*/
	//line(lv1, Point(xm, ym), Point(xm, ym), Scalar(128, 128, 255), 5);
	//imshow("lv1", lv1); waitKey(0);*/
	/************************************************/
	Mat lb = Mat::zeros(h, w, CV_32FC1);
	Mat rb = Mat::zeros(h, w, CV_32FC1);
	Mat tb = Mat::zeros(h, w, CV_32FC1);
	//draw bisectors
	btgr = (float *)bt.data, tlgr = (float *)tl.data, trgr = (float *)tr.data;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, btgr += 4, tlgr += 4, trgr += 4){
			if (*btgr){
				// find left vertex
				int i1_min = (i > BI_max) ? (i - BI_max) : 0, j1_min = (j > R_max) ? (j - R_max) : 0, \
					i1_max = (i + 20 < h) ? (i + 20) : h, j1_max = (j + R_max_2 < w) ? (j + R_max_2) : w;
				float *tlgr_mag_off = (float *)tl.data + 4 * (i1_min*w + j1_min);
				/*float *tlgr_tan_off = tlgr_mag_off + 2;*/
				for (int i1 = i1_min; i1 < i1_max; i1++, tlgr_mag_off += 4 * w){
					float *tl_mg = tlgr_mag_off/*, *tl_tan = tl_mg + 2*/;
					for (int j1 = j1_min; j1 < j1_max; j1++, tl_mg += 4/*, tl_tan += 4*/){
						if (!(*tl_mg)) continue;
						if (!top_hat(sqrt((i - i1)*(i - i1) + (j - j1)*(j - j1)), R_max)) continue;
						if ((i <= i1 + (*(tl_mg + 2)) * (j - j1) + VERTEX_EPSILON) || \
							(i1 >= i + (*(btgr + 2)) * (j1 - j) - VERTEX_EPSILON))
							continue;

						float ai = *(btgr + 2), aj = *(tl_mg + 2);
						float xA = (i1 - aj*j1 - i + ai*j) / (ai - aj);
						float yA = ai*(xA - j) + i;
						if ((yA < 20) || (xA > w - 25) || (xA <= 0) || (yA >= h-1)) continue;
						float vertexscore = 0; int xAmax, yAmax;
						for (int y = yA - 2; y <= min((int)xA + 2, h - 1); y++)
						for (int x = max(0, (int)xA - 2); x <= xA + 2; x++){
							if (lv1.at<float>(y, x) > vertexscore){
								vertexscore = lv1.at<float>(y, x);
								xAmax = x; yAmax = y;
							}
						}
						if (vertexscore == 0) continue;
						int xB = min(w - 1, xAmax + A_CENTER_max_x/*(int)(sqrt(vertexscore/2) * cos30)*/);
						float cosnj = grad_x.at<float>(i1, j1), cosni = grad_x.at<float>(i, j), \
							sinnj = grad_y.at<float>(i1, j1), sinni = grad_y.at<float>(i, j);
						int yB = ((cosnj - cosni) * xB + j*cosni + i*sinni - j1*cosnj - i1*sinnj)/(sinni - sinnj);
						vote(Point(xAmax, yAmax), Point(xB, yB), 1/*(*(btgr + 3))*(*(tl_mg + 3))*/, lb);		//  check if this added more to point A (check by debug!)
						/*if (*(btgr + 3) == 0)
							(*(btgr + 3)) = log10(1 + (*(btgr)));
						if (*(tlgr_mag_off + 3) == 0)
							*(tlgr_mag_off + 3) = log10(1 + (*(tlgr_mag_off + j1)));*/
					}
				}

				// find right vertex
				i1_min = (i > BI_max) ? (i - BI_max) : 0; j1_min = (j > R_max_2) ? (j - R_max_2) : 0;
				i1_max = (i + 20 < h) ? (i + 20) : h; j1_max = (j + R_max < w) ? (j + R_max) : w;
				float *trgr_mag_off = (float *)tr.data + 4 * (i1_min*w + j1_min);
				/*float *trgr_tan_off = trgr_mag_off + 2;*/
				for (int i1 = i1_min; i1 < i1_max; i1++, trgr_mag_off += 4 * w){
					float *tr_mg = trgr_mag_off;
					for (int j1 = j1_min; j1 < j1_max; j1++, tr_mg += 4){
						if (!*(tr_mg)) continue;
						if (!top_hat(sqrt((i - i1)*(i - i1) + (j - j1)*(j - j1)), R_max)) continue;
						if ((i1 >= i + (*(btgr + 2)) * (j1 - j) - VERTEX_EPSILON) || \
							(i <= i1 + (*(tr_mg + 2)) * (j - j1) + VERTEX_EPSILON))
							continue;

						float ai = *(btgr + 2), aj = *(tr_mg + 2);
						float xA = (i1 - aj*j1 - i + ai*j) / (ai - aj);
						float yA = ai*(xA - j) + i;

						if ((yA < 20) || (xA < 25) || (xA >= w-1) || (yA >= h-1)) continue;
						float vertexscore = 0; int xAmax, yAmax;
						for (int y = yA - 2; y <= min((int)yA + 2, h - 1); y++)
						for (int x = xA - 2; x <= min((int)xA + 2, w - 1); x++){
							if (rv1.at<float>(y, x) > vertexscore){
								vertexscore = rv1.at<float>(y, x);
								xAmax = x; yAmax = y;
							}
						}
						if (vertexscore == 0) continue;
						int xB = max(0, xAmax - A_CENTER_max_x/*(int)(sqrt(vertexscore / 2) * cos30)*/);
						float cosnj = grad_x.at<float>(i1, j1), cosni = grad_x.at<float>(i, j), \
							sinnj = grad_y.at<float>(i1, j1), sinni = grad_y.at<float>(i, j);
						int yB = ((cosnj - cosni) * xB + j*cosni + i*sinni - j1*cosnj - i1*sinnj) / (sinni - sinnj);
						vote(Point(xAmax, yAmax), Point(xB, yB), 1/*(*(btgr + 3))*(*(tr_mg + 3))*/, rb);		// check if this add more to point A (check by debug!)
						/*if (*(btgr + 3) == 0)
							(*(btgr + 3)) = log10(1 + (*(btgr)));
						if (*(trgr_mag_off + 3) == 0)
							*(trgr_mag_off + 3) = log10(1 + (*(trgr_mag_off + j1)));*/
					}
				}
			}
		}
	}

	tlgr = (float *)tl.data/*, trgr = (float *)tr.data*/;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, tlgr += 4/*, trgr += 4*/){
			if (*tlgr){
				// find top vertex
				int i1_min = (i > BI_max) ? (i - BI_max) : 0, \
					i1_max = (i + BI_max < h) ? (i + BI_max) : h, j1_max = (j + R_max < w) ? (j + R_max) : w;
				float *trgr_mag_off = (float *)tr.data + 4 * (i1_min*w + j);

				for (int i1 = i1_min; i1 < i1_max; i1++, trgr_mag_off += 4 * w){
					float *tr_mg = trgr_mag_off;
					for (int j1 = j; j1 < j1_max; j1++, tr_mg += 4){
						if (!*(tr_mg)) continue;
						if (!top_hat(sqrt((i - i1)*(i - i1) + (j - j1)*(j - j1)), R_max)) continue;
						if ((i1 <= i + (*(tlgr + 2)) * (j1 - j) + VERTEX_EPSILON) || \
							(i <= i1 + (*(tr_mg + 2)) * (j - j1) + VERTEX_EPSILON))
							continue;

						float ai = *(tlgr + 2), aj = *(tr_mg + 2);
						float xA = (i1 - aj*j1 - i + ai*j) / (ai - aj);
						float yA = ai*(xA - j) + i;

						if ((yA > h - 20) || (xA <= 12) || (xA >= w - 13) || (yA <= 0)) continue;
						float vertexscore = 0; int xAmax, yAmax;
						for (int y = max((int)yA - 2, 0); y <= yA + 2; y++)
						for (int x = xA - 2; x <= xA + 2; x++){
							if (tv1.at<float>(y, x) > vertexscore){
								vertexscore = tv1.at<float>(y, x);
								xAmax = x; yAmax = y;
							}
						}
						if (vertexscore == 0) continue;
						int xB, yB = min(w - 1, yAmax + A_CENTER_max/*(int)sqrt(vertexscore / 2)*/);
						float cosnj = grad_x.at<float>(i1, j1), cosni = grad_x.at<float>(i, j), \
							sinnj = grad_y.at<float>(i1, j1), sinni = grad_y.at<float>(i, j);
						if (sinni == sinnj) xB = xAmax;
						else xB = ((sinnj - sinni) * yB + j*cosni + i*sinni - j1*cosnj - i1*sinnj) / (cosni - cosnj);
						vote(Point(xAmax, yAmax), Point(xB, yB), 1/*(*(tlgr + 3))*(*(tr_mg + 3))*/, tb);		// check if this add more to point A (check by debug!)
						/*if (*(tlgr + 3) == 0)
							(*(tlgr + 3)) = log10(1 + (*(tlgr)));
						if (*(trgr_mag_off + 3) == 0)
							*(trgr_mag_off + 3) = log10(1 + (*(trgr_mag_off + j1)));*/
					}
				}
			}
		}
	}

	/*********************************Test*********************************/
	/*double maxim;
	minMaxLoc(tb, NULL, &maxim);
	tb *= 1.0f / maxim;
	minMaxLoc(lb, NULL, &maxim);
	lb *= 1.0f / maxim;
	minMaxLoc(rb, NULL, &maxim);
	rb *= 1.0f / maxim;
	imshow("tb", tb);
	imshow("lb", lb);
	imshow("rb", rb);
	waitKey(0);
	Mat bisectors = rb + lb + tb;
	minMaxLoc(bisectors, NULL, &maxim);
	bisectors *= 1.0f / maxim;
	imshow("bisectors", bisectors);
	waitKey(0);*/
	/*********************************Test*********************************/
	Mat center = Mat::zeros(h, w, CV_32FC1);
	// check in-center score
	float *topb = (float *)tb.data, *leftb = (float *)lb.data, *rightb = (float *)rb.data;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, topb++, leftb++, rightb++){
			if (*topb > VERTEX_THRESHOLD && *leftb > VERTEX_THRESHOLD && *rightb > VERTEX_THRESHOLD \
				&& ((*topb) + (*leftb) + (*rightb)) > INCENTER_THRESHOLD)
				center.at<float>(i, j) = ((*topb) + (*leftb) + (*rightb));
		}
	}
	float *c = (float *)center.data;
	for (int i = 0; i < h; i++){
		for (int j = 0; j < w; j++, c++){
			int tvx = 0, tvy = 0, lvx = 0, lvy = 0, rvx = 0, rvy = 0, \
				i1max = (i < h - 15) ? (i + 15) : (h - 1), j1max = (j < w - 15) ? (j + 15) : (w - 1);
			for (int i1 = (i > 15) ? (i - 15) : 0; i1 <= i1max; i1++){
				for (int j1 = (j > 15) ? (j - 15) : 0; j1 <= j1max; j1++){
					if (*c < center.at<float>(i1,j1))
						*c = 0;
				}
			}
			if (*c){
				//top vertex
				for (int iv = max(i - 17, 0); iv >= max(i - 92, 0); iv--)
				for (int jv = max(j - 25, 0); jv <= min(j + 25, w - 1); jv++)
				if (tv1.at<float>(iv, jv))
				{
					tvx = jv;
					tvy = iv;
					iv = 0; jv = w;
				}

				//left vertex
				for (int jv = max(j - 12, 0); jv >= max(j - 80, 0); jv--){
					for (int iv = min(i + 8, h - 1); iv <= min(i + 46, h - 1); iv++){
						if (lv1.at<float>(iv, jv))
						{
							lvx = jv;
							lvy = iv;
							iv = h;
							jv = 0;
						}
					}
				}

				//right vertex
				for (int jv = min(j + 12, w - 1); jv <= min(j + 80, w - 1); jv++)
				for (int iv = min(i + 8, h - 1); iv <= min(i + 46, h - 1); iv++)
				if (rv1.at<float>(iv, jv))
				{
					rvx = jv;
					rvy = iv;
					jv = w; iv = h;
				}

				ret.push_back(new Triangle(Point(j, i), Point(tvx, tvy), Point(lvx, lvy), Point(rvx, rvy)));
			}
		}
	}
	t1 = clock();
	return ret;
}
