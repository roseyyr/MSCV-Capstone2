/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);
set<int> LoadMask(const string &mask_path, cv::Mat &mask);

set<int> dynamic_classes = {0,1,2,3,4,5,6,7,8,14,15,16,17,18,19,20,21,22,23};
int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    std::cout<<"nImages:"<<nImages<<std::endl;
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;
    cv::Mat mask(480,640,CV_8U);
    cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(31,31));
  
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        string mask_path = string(argv[3])+"/segment/"+vstrImageFilenamesRGB[ni].substr(4,17)+".msk";
    	std::set<int> dynamic_instances = LoadMask(mask_path,mask);
	   // Form a mask that discriminate dynamics
	   /*
	   cv::Mat object_mask = mask;
	   int cnt = 0;
       for(int i=0;i<480;i++)
       {
            for(int j=0;j<640;j++)
            {
                int label = mask.at<uchar>(i,j);
                if(label == 1)
                {
                    cnt ++;
                }
            }
        }
        */
	
        cv::Mat object_mask = cv::Mat::ones(480,640,CV_8U);
	int cnt = 0;
        for(int i=0;i<480;i++)
        {
            for(int j=0;j<640;j++)
            {
                int label = mask.at<uchar>(i,j);
                if(dynamic_instances.count(label))
                {
                    object_mask.at<uchar>(i,j) = 1;
                    cnt ++;
                }
                else
                    object_mask.at<uchar>(i,j) = 0;
            }
        }

        cout<<cnt<<" before dilation cnt"<<endl;
        // Dilate the mask
        cv::Mat mask_dilated = cv::Mat::zeros(480,640,CV_8U);
        cv::dilate(object_mask,mask_dilated,kernel);
        cv::Mat one = cv::Mat::ones(480,640,CV_8U);
        cv::Mat new_mask = one - mask_dilated;
        // Erode the mask
        cv::Mat mask_eroded = new_mask.clone();
        cv::erode(new_mask,mask_eroded,kernel);
        cnt = 0;
        for(int i=0;i<480;i++)
        {
            for(int j=0;j<640;j++)
            {
                int label = mask_eroded.at<uchar>(i,j);
                if(label == 0)
                {
                    cnt++;
                }
            }
        }
        cout<<cnt<<" after erosion cnt"<<endl;


        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        //cout<<"before tracking..."<<endl;
        // Pass the image to the SLAM system
        SLAM.ObjectTrackRGBD(imRGB,imD,tframe,mask,dynamic_instances,mask_eroded);
	//cout<<"after tracking..."<<endl;

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}
std::set<int> LoadMask(const string &mask_path, cv::Mat &mask)
{
    int H=480,W=640;
    ifstream fMask;
    fMask.open(mask_path.c_str());
    for(int i=0;i<H;i++)
    {
	//cout<<i<<endl;
        string s;
	getline(fMask,s);
	stringstream ss;
	ss << s;
	int t;
	for(int j=0;j<W;j++)
	{
            ss >> t;
	    mask.at<uchar>(i,j) = t;
	}
	//cout<<endl;
    }
         
    string s;
    getline(fMask,s);
    stringstream ss;
    ss << s;
    int object_num;
    ss >> object_num;
    cout<<"number of instances:"<<object_num<<endl;
    std::set<int> dynamic_instances;
    for(int i=0;i<object_num;i++)
    {
        string s;
    	getline(fMask,s);
	   stringstream ss;
	   ss << s;

	   int instance_id, category_id;
	   int isThing;
	   float score;
	   ss >> instance_id;
	   ss >> isThing;
	   //cout<<"instance id:"<<instance_id<<" isThing:"<<isThing<<endl;
	   if(isThing)
	   {
	        ss >> category_id;
	       ss >> score;
	       if(dynamic_classes.count(category_id))
	       {
	           dynamic_instances.insert(instance_id);
		      cout<<"dynamic object:"<<instance_id<<endl;
	       }
	   }
	   else
	   {
	       ss >> category_id;
	   }
    }
    return dynamic_instances;
}
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    { 
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
