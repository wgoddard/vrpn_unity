/* ========================================================================
 * PROJECT: UART
 * ========================================================================
 * Portions of this work are built on top of the ARToolkitPlus, 
 * which is in turn based on the original ARToolKit developed by
 *   Hirokazu Kato
 *   Mark Billinghurst
 *   HITLab, University of Washington, Seattle
 * http://www.hitl.washington.edu/artoolkit/
 *
 * Portions of this work are also built on top of the VideoWrapper,
 * a BSD licensed video access library for MacOSX and Windows.
 * VideoWrapper is available at SourceForge via 
 * http://sourceforge.net/projects/videowrapper/
 *
 * Copyright of the ARToolkitPlus is
 *     (C) 2006 Graz University of Technology
 *
 * Copyright of VideoWrapper is
 *     (C) 2003-2010 Georgia Tech Research Corportation
 *
 * Copyright of the new and derived portions of this work
 *     (C) 2010 Georgia Tech Research Corporation
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this framework; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For further information regarding UART, please contact 
 *   Blair MacIntyre
 *   <blair@cc.gatech.edu>
 *   Georgia Tech, School of Interactive Computing
 *   85 5th Street NW
 *   Atlanta, GA 30308
 *
 * For further information regarding ARToolkitPlus, please contact 
 *   Dieter Schmalstieg
 *   <schmalstieg@icg.tu-graz.ac.at>
 *   Graz University of Technology, 
 *   Institut for Computer Graphics and Vision,
 *   Inffeldgasse 16a, 8010 Graz, Austria.
 *
 * ========================================================================
 ** @author   Alex Hill (ahill@gatech.edu)
 *
 * ======================================================================== */
 
#if _MSC_VER // this is defined when compiling with Visual Studio
#define EXPORT_API __declspec(dllexport) // Visual Studio needs annotating exported functions with this
#else
#define EXPORT_API // XCode does not need annotating exported functions, so define is empty
#endif

#include <math.h>
#include <string>
#include <map>
using std::string;

#include "ARToolkitPlus/ar.h"
#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"
#include "ARToolKitPlus/TrackerMultiMarkerImpl.h"

// ------------------------------------------------------------------------
// Plugin itself

#define MAX_NUM_TRACKERS 10

// Link following functions C-style (required for plugins)
extern "C"
{
	
	typedef	struct _tw_TRACKERCB {
		int	id;
		int type;
		float pos[3];
		float quat[4];
		float conf;
		float corners[8];
		int area;
		char* data;
	} tw_TRACKERCB;
	
	class tw_Tracker
	{
	public:
		ARToolKitPlus::TrackerSingleMarker* single;
		ARToolKitPlus::TrackerMultiMarker* multi;
		ARToolKitPlus::Profiler* profiler;
		char calibration[256];
		char markername[256];
		int threshold;
		int singleMode;
		int multiMode;
		bool estimate;
		bool learn;
		std::map<int,float> widths;
		tw_Tracker() 
		{ 
			single = NULL;
			multi = NULL;
			profiler = NULL;
			threshold = 0;
			singleMode = (int)ARToolKitPlus::MARKER_ID_SIMPLE;
			multiMode = (int)ARToolKitPlus::MARKER_ID_SIMPLE;
			estimate = true;
		};
		~tw_Tracker()
		{
			if (single) delete single;
			if (multi) delete multi;
			if (profiler) delete profiler;
		};
	};
	
	static tw_Tracker* trackers[MAX_NUM_TRACKERS] = {0};
	
	class ARTKLogger : public ARToolKitPlus::Logger
	{
		public:
		void artLog(const char* nMessage)
		{
			printf(nMessage);
			printf("\n");

			#ifdef WIN32
			FILE* fp = fopen("log.txt", "a");
			fprintf(fp, "%s\n", nMessage);
			fclose(fp);
			#endif
			
		}
	};

	static ARTKLogger myLogger;	

	enum PIXEL_FORMAT {
		PIXEL_FORMAT_UNKNOWN = 0,
		PIXEL_FORMAT_ABGR = 1,
		PIXEL_FORMAT_BGRA = 2,
		PIXEL_FORMAT_BGR = 3,
		PIXEL_FORMAT_RGBA = 4,
		PIXEL_FORMAT_RGB = 5,
		PIXEL_FORMAT_RGB565 = 6,
		PIXEL_FORMAT_LUM = 7,
		PIXEL_FORMAT_RGBA5551 = 8,
		PIXEL_FORMAT_RGBA4444 = 9,
		PIXEL_FORMAT_PALETTED = 10
	};
	
	enum CONFIG_FLAGS {
		T_SIMPLEID = 1,
		T_BCH = 2,
		T_FRAME_SIMPLEID = 4,	
		T_SPLIT = 8,
		T_DATAMATRIX = 16,
		T_TEMPLATE = 32,
		T_NULL = 1024,
		T_GN = 2048,
		T_RPP = 4096,
		T_MULTI = 8192,
		T_FIXED = 16384,
		T_THICKBORDER = 32768,
		T_SINGLE = 65536,
		T_NONUNIFORM = 1048576,
		T_POSEFILTER = 2097152,
		T_CORNERFILTER =  4194304,
		T_DUMPDATA = 8388608,
		T_PROFILER = 16777216,
		T_HALFIMAGE = 33554432,
		T_FASTMODE = 67108864
	};
	
	int EXPORT_API TRACKER_setThreshold(int handle, int val)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		trackers[handle]->threshold = val;
		if (trackers[handle]->single)
		{
			trackers[handle]->single->setThreshold(val);
			trackers[handle]->single->activateAutoThreshold(false);
		}
		if (trackers[handle]->multi)
		{
			trackers[handle]->multi->setThreshold(val);
			trackers[handle]->multi->activateAutoThreshold(false);
		}
		return 0;
	}
	
	int EXPORT_API TRACKER_setBorder(int handle, float borderWidth, int flags)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		if (trackers[handle]->single)
			trackers[handle]->single->setBorderWidth(borderWidth);
		if (trackers[handle]->multi)
			trackers[handle]->multi->setBorderWidth(borderWidth);
		return 0;
	}
	
	int EXPORT_API TRACKER_setLearnNew(int handle, bool learn, float width, float height, int flags)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		trackers[handle]->learn = learn;
		trackers[handle]->widths[-1] = width;
		return 0;
	}
	
	int EXPORT_API TRACKER_registerSingleMarker(int handle, const char* markername, int id, const char* data, float width, float height, int flags)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		tw_Tracker* tracker = trackers[handle];
		if (!tracker->single)
		{
			if(flags&T_FRAME_SIMPLEID)
				tracker->single = new ARToolKitPlus::TrackerSingleMarkerImpl<20,20,20, 1, 16>(320,240);
			else
				tracker->single = new ARToolKitPlus::TrackerSingleMarkerImpl<6,6,6, 1, 16>(320,240);
			if (flags&T_DUMPDATA)
				tracker->single->setDumpData(true);
			else
				tracker->single->setDumpData(false);
			if(!tracker->single->init(tracker->calibration, 1.0f, 1000.0f))
				myLogger.artLog("Failed to load camera file");
			else
				myLogger.artLog("Loaded camera file");
			if(flags&T_HALFIMAGE)
				myLogger.artLog("Using half image processing");
			else
				tracker->single->setImageProcessingMode(ARToolKitPlus::IMAGE_FULL_RES);
			if (tracker->threshold > 0)
			{
				tracker->single->setThreshold(tracker->threshold);
				tracker->single->activateAutoThreshold(false);
			}
			else
				tracker->single->activateAutoThreshold(true);
			if (flags&T_NULL)
			{
				tracker->estimate = false;
				myLogger.artLog("Registered PoseEstimatorNull");
			}
			else
			{
				if(flags&T_RPP)
				{
					tracker->single->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
					myLogger.artLog("Registered PoseEstimatorRPP");
				}
				else 
				{
					myLogger.artLog("Registered PoseEstimatorGN");
					if (flags&T_POSEFILTER)
					{
						tracker->single->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL_CONT);
						myLogger.artLog("Registered PoseFilter");
					}
					else
						tracker->single->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL);
				}
			}
		}
		if (flags&T_TEMPLATE)
		{
			if(flags&T_THICKBORDER)
				tracker->single->setBorderWidth(0.250f);
			else
				tracker->single->setBorderWidth(0.125f);
			tracker->single->setMarkerMode(ARToolKitPlus::MARKER_TEMPLATE);
			tracker->singleMode = (int)ARToolKitPlus::MARKER_TEMPLATE;
			tracker->single->addPattern(data);
			myLogger.artLog("Creating MarkerTemplate");
			myLogger.artLog("Registered SingleMarkerTarget");
		}
		else if (flags&T_BCH)
		{
			if(flags&T_THICKBORDER)
				tracker->single->setBorderWidth(0.250f);
			else
				tracker->single->setBorderWidth(0.125f);
			tracker->single->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);
			tracker->singleMode = (int)ARToolKitPlus::MARKER_ID_BCH;
			myLogger.artLog("Creating MarkerBCH");
			myLogger.artLog("Registered SingleMarkerTarget");
		}
		else if (flags&T_SIMPLEID)
		{
			if(flags&T_THICKBORDER)
				tracker->single->setBorderWidth(0.250f);
			else
				tracker->single->setBorderWidth(0.125f);
			tracker->single->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
			tracker->singleMode = (int)ARToolKitPlus::MARKER_ID_SIMPLE;
			myLogger.artLog("Creating MarkerSimpleId");
			myLogger.artLog("Registered SingleMarkerTarget");
		}
		else if (flags&T_FRAME_SIMPLEID)
		{
			if(flags&T_THICKBORDER)
				tracker->single->setBorderWidth(0.0909f);
			else
				tracker->single->setBorderWidth(0.04545f);
			tracker->single->setMarkerMode(ARToolKitPlus::MARKER_ID_FRAME_SIMPLE);
			tracker->singleMode = (int)ARToolKitPlus::MARKER_ID_FRAME_SIMPLE;
			myLogger.artLog("Creating MarkerFrameSimpleId");
			myLogger.artLog("Registered SingleMarkerTarget");
		}
		if (id < 0)
			tracker->learn = true;
		tracker->widths[id] = width;
		return 0;
	}
	
	int EXPORT_API TRACKER_registerMultiMarker(int handle, const char* markername, const char* filename, int flags)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		tw_Tracker* tracker = trackers[handle];
		if (!tracker->multi)
		{
			tracker->multi = new ARToolKitPlus::TrackerMultiMarkerImpl<6,6,6, 1, 16>(320,240);
			if (flags&T_DUMPDATA)
				tracker->multi->setDumpData(true);
			else
				tracker->multi->setDumpData(false);
			if(!tracker->multi->init(tracker->calibration, filename, 1.0f, 1000.0f))
				myLogger.artLog("Failed to load multi marker file");
			else
				myLogger.artLog("Loaded multi marker file");
			if (flags&T_FASTMODE)
			{
				myLogger.artLog("Using Fast Mode");
				tracker->multi->setUseDetectLite(true);
			}
			else
				tracker->multi->setUseDetectLite(false);
		}
		strcpy(tracker->markername,markername);
		if (tracker->threshold > 0)
		{
			tracker->multi->setThreshold(tracker->threshold);
			tracker->multi->activateAutoThreshold(false);
		}
		else
			tracker->multi->activateAutoThreshold(true);
		if (flags&T_BCH)
		{
			if(flags&T_THICKBORDER)
				tracker->multi->setBorderWidth(0.250f);
			else
				tracker->multi->setBorderWidth(0.125f);
			tracker->multiMode = (int)ARToolKitPlus::MARKER_ID_BCH;
			tracker->multi->setMarkerMode(ARToolKitPlus::MARKER_ID_BCH);
			myLogger.artLog("Registered MarkerDetectorBCH");
		}
		else
		{
			if(flags&T_THICKBORDER)
				tracker->multi->setBorderWidth(0.250f);
			else
				tracker->multi->setBorderWidth(0.125f);
			tracker->multiMode = (int)ARToolKitPlus::MARKER_ID_SIMPLE;
			tracker->multi->setMarkerMode(ARToolKitPlus::MARKER_ID_SIMPLE);
			myLogger.artLog("Registered MarkerDetectorSimpleId");
		}
		if (flags&T_NULL)
		{
			tracker->estimate = false;
			myLogger.artLog("Registered PoseEstimatorNull");
		}
		else
		{
			if(flags&T_RPP)
			{
				tracker->multi->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_RPP);
				myLogger.artLog("Registered PoseEstimatorRPPMulti");
			}
			else 
			{
				myLogger.artLog("Registered PoseEstimatorGNMulti");
				if (flags&T_POSEFILTER)
				{
					tracker->multi->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL_CONT);
					myLogger.artLog("Registered PoseFilter");
				}
				else
					tracker->multi->setPoseEstimator(ARToolKitPlus::POSE_ESTIMATOR_ORIGINAL);
			}
		}
		myLogger.artLog("Registered MultiMarkerTarget");
		return 0;
	}
	
	int EXPORT_API TRACKER_createTracker(const char* calibration, int flags, int& handle)
	{
		handle = -1;
		for (int i=0;i<MAX_NUM_TRACKERS;i++)
			if (trackers[i] == NULL)
			{
				handle = i;
				break;
			}
		if (handle >= 0)
		{
			trackers[handle] = new tw_Tracker();
			if (flags&T_PROFILER)
				trackers[handle]->profiler = new ARToolKitPlus::Profiler();
			strcpy(trackers[handle]->calibration,calibration);
			return 0;
		}
		return -1;
	}

	int EXPORT_API TRACKER_deleteTracker(int handle)
	{
		if (handle >= 0 && handle < MAX_NUM_TRACKERS && trackers[handle])
		{
			delete trackers[handle];
			trackers[handle] = NULL;
			return 0;
		}
		return -1;
	}
		
	void ARTKMatrixToQuat(const float m[3][4], float* q)
	{
		float tr, s;
		int i, j, k;
		int nxt[3] = {1, 2, 0};
		
		tr = m[0][0] + m[1][1] + m[2][2];
		
		// check the diagonal
		if (tr > 0.0)
		{
			s = sqrt(tr + 1.0f);
			q[3] = s / 2.0f;
			s = 0.5f / s;
			q[0] = (m[2][1] - m[1][2]) * s;
			q[1] = (m[0][2] - m[2][0]) * s;
			q[2] = (m[1][0] - m[0][1]) * s;
		}
		else
		{
			// diagonal is negative
			i = 0;
			if (m[1][1] > m[0][0]) i = 1;
			if (m[2][2] > m[i][i]) i = 2;
			j = nxt[i];
			k = nxt[j];

			s = sqrt((m[i][i] - (m[j][j] + m[k][k])) + 1.0f);
			
			q[i] = s * 0.5f;
			
			if(s != 0.0f)
				s = 0.5f / s;
			
			q[3] = (m[k][j] - m[j][k]) * s;
			q[j] = (m[j][i] + m[i][j]) * s;
			q[k] = (m[k][i] + m[i][k]) * s;
		}
	}
		
	int EXPORT_API TRACKER_getReportsChar(int handle, const unsigned char* cameraBuffer, int width, int height, int format, tw_TRACKERCB** cpy,int& cnt)
	{
		cnt = 0;
		int marker_cnt = 0;
		ARToolKitPlus::ARMarkerInfo *marker_info = NULL;
		ARToolKitPlus::ARMultiMarkerInfoT* config = NULL;
		ARToolKitPlus::TrackerMultiMarker* multi = trackers[handle]->multi;
		PROFILE_BEGINSEC(profiler, SINGLEMARKER_OVERALL)
		if (multi)
		{
			multi->setPixelFormat((ARToolKitPlus::PIXEL_FORMAT)format);
			multi->changeCameraSize(width,height);
			if (multi->arDetectMarker(const_cast<unsigned char*>(cameraBuffer), multi->getThreshold(), &marker_info, &marker_cnt) < 0)
				return -1;
			config = (ARToolKitPlus::ARMultiMarkerInfoT*)multi->getMultiMarkerConfig();
			if (trackers[handle]->estimate && multi->executeMultiMarkerPoseEstimator(marker_info, marker_cnt, config) >= 0)
			{
				(*cpy[cnt]).conf = 1.0;
				(*cpy[cnt]).area = 0.0;
				strcpy((*cpy[cnt]).data,trackers[handle]->markername);
				(*cpy[cnt]).type = T_MULTI;
				(*cpy[cnt]).pos[0] = config->trans[0][3];
				(*cpy[cnt]).pos[1] = config->trans[1][3];
				(*cpy[cnt]).pos[2] = config->trans[2][3];
				ARTKMatrixToQuat(config->trans, (*cpy[cnt]).quat);
				cnt++;
			}
		}
		ARToolKitPlus::TrackerSingleMarker* single = trackers[handle]->single;
		if (single)
		{
			if (multi && trackers[handle]->singleMode == trackers[handle]->multiMode)
			{
				for(int j=0; j<marker_cnt; j++) 
				{
					for(int k=0; k<config->marker_num; k++) 
						if(marker_info[j].id == config->marker[k].patt_id)
						{
							marker_info[j].id = -1;
							break;
						}
				}
			}
			else
			{
				single->setPixelFormat((ARToolKitPlus::PIXEL_FORMAT)format);
				single->changeCameraSize(width,height);
				if (single->arDetectMarker(const_cast<unsigned char*>(cameraBuffer), single->getThreshold(), &marker_info, &marker_cnt) < 0)
					return -1;
			}
			for (int i=0;i<marker_cnt;i++)
			{
				if (marker_info[i].id >= 0)
				{
					(*cpy[cnt]).conf = marker_info[i].cf;
					(*cpy[cnt]).area = marker_info[i].area;
					int dir = marker_info[i].dir;
					for (int j=0;j<4;j++)
					{
						(*cpy[cnt]).corners[2*j] = marker_info[i].vertex[(j+4-dir)%4][0];
						(*cpy[cnt]).corners[2*j+1] = marker_info[i].vertex[(j+4-dir)%4][1];
					}
					(*cpy[cnt]).id = marker_info[i].id;
					(*cpy[cnt]).type = trackers[handle]->singleMode;
					ARFloat patt_center[2];
					patt_center[0] = patt_center[1] = 0.0;
					ARFloat patt_width = trackers[handle]->widths[marker_info[i].id];
					if (patt_width == 0.0)
						patt_width = trackers[handle]->widths[-1];
					ARFloat trans[3][4];
					if (!trackers[handle]->estimate || single->executeSingleMarkerPoseEstimator(&marker_info[i], patt_center, patt_width, trans) < 0)
						continue;
					(*cpy[cnt]).pos[0] = trans[0][3];
					(*cpy[cnt]).pos[1] = trans[1][3];
					(*cpy[cnt]).pos[2] = trans[2][3];
					ARTKMatrixToQuat(trans, (*cpy[cnt]).quat);
					cnt++;
				}
			}
		}
		PROFILE_ENDSEC(profiler, SINGLEMARKER_OVERALL)
		return 0;			
	}
	
	int  EXPORT_API TRACKER_getReportsFile(int handle, const char* fName, int width, int height, int format, tw_TRACKERCB** dst,int& cnt)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		trackers[handle]->single->setPixelFormat((ARToolKitPlus::PIXEL_FORMAT)format);
		int bpp = trackers[handle]->single->getBitsPerPixel()/8;
		size_t numPixels = width*height*bpp;
		size_t numBytesRead;
		
		unsigned char *cameraBuffer = new unsigned char[numPixels];
		
		if(FILE* fp = fopen(fName, "rb"))
		{
			numBytesRead = fread(cameraBuffer, 1, numPixels, fp);
			fclose(fp);
		}
		else
		{
			printf("Failed to open %s\n", fName);
			delete [] cameraBuffer;
		}
		
		if(numBytesRead != numPixels)
		{
			printf("Failed to read %s\n", fName);
			delete [] cameraBuffer;
		}
		return TRACKER_getReportsChar(handle, cameraBuffer,width,height,format,dst,cnt);
	}
	
	int EXPORT_API TRACKER_getReports(int handle, void* src, int width, int height, int format, tw_TRACKERCB** dst,int& cnt)
	{
		if (handle < 0 || handle >= MAX_NUM_TRACKERS || trackers[handle] == NULL)
			return -1;
		unsigned char *cameraBuffer = (unsigned char*)(src);
		return TRACKER_getReportsChar(handle,cameraBuffer,width,height,format,dst,cnt);
	}
	
}
