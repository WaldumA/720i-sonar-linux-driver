#include "sonar_processor_node.hpp"

// Function visualising the processed data vs raw data. This function is only for testing purposes.
void visualisingTest(MatrixXi sonarData, rangeAndAngle rangeAndAngles, int num_bins, int num_beams, float max_range) {
    // Necesarry variables
    int width, height, scale;
    MatrixXi tmp_raw_image, tmp_processed_image;
    float tmp_width, tmp_height, angle, range;
    Mat tmp_processed_cv_image, tmp_raw_cv_image, tmp_processed_cv_image_gray, tmp_raw_cv_image_gray;

    // Initialising 
    width = 800;
    height = 800;
    scale = 200/(max_range);
    tmp_raw_image = MatrixXi(height,width);
    tmp_raw_image.setConstant(255);
    tmp_processed_image = MatrixXi(height,width);
    tmp_processed_image.setConstant(255);

    
    // Drawing processed data
    for (int i = 0; i < rangeAndAngles.range.size(); i++) {
        if (range != -1) {
            tmp_width = sin(rangeAndAngles.angle[i])*rangeAndAngles.range[i];
            tmp_height = sqrt(pow(rangeAndAngles.range[i],2.0) - pow(tmp_width,2.0));
            tmp_width = (width/2) - (tmp_width*scale);
            tmp_height = (height-5) - (tmp_height*scale);
            tmp_processed_image((int)tmp_height,(int)tmp_width) = 0;
        }
    }
    
    // Drawing raw data
    for (int i = 0; i < num_beams; i++) {
        angle = asin((float((2*i-256))/256.0)*0.86602540); // Formula given by tritech
        for (int j = 0; j < num_bins; j++) {
            range = max_range * (float(j+1)/float(num_bins));
            tmp_width = sin(angle)*range;
            tmp_height = sqrt(pow(range,2.0) - pow(tmp_width,2.0));
            tmp_width = (width/2) - (tmp_width*scale);
            tmp_height = (height-5) - (tmp_height*scale);
            
            tmp_raw_image((int)tmp_height,(int)tmp_width) = (int)(255 - float(sonarData(i,j))/float(256.0)*float(255));
        }
    }

    // Visualising
    eigen2cv(tmp_processed_image,tmp_processed_cv_image);
    tmp_processed_cv_image.convertTo(tmp_processed_cv_image_gray, CV_8U);
    eigen2cv(tmp_raw_image,tmp_raw_cv_image);
    tmp_raw_cv_image.convertTo(tmp_raw_cv_image_gray, CV_8U);

    namedWindow("Processed data", 0);
    resizeWindow("Processed data", 1200, 1200);
    imshow("Processed data", tmp_processed_cv_image_gray);
    namedWindow("Raw data", 0);
    resizeWindow("Raw data", 1200, 1200);
    imshow("Raw data", tmp_raw_cv_image_gray);
    waitKey(5);
}

// Function taking the ranges and angels found for the best beams and publishes them in the sonar_processed_data format
sonar_msgs::sonar_processed_data createMsg(rangeAndAngle rangeAndAngles, rangeAndAngle thresholdedRangeandAngles) {
    // Necesarry variables
    sonar_msgs::sonar_processed_data msg;

    // Looping through rangeAndAngle struct filling up the message
    for (int i = 0; i < rangeAndAngles.range.size();i++) {
        msg.range.push_back(rangeAndAngles.range[i]);
        msg.angles.push_back(rangeAndAngles.angle[i]);
        msg.thresholdAngles.push_back(thresholdedRangeandAngles.angle[i]);
        msg.thresholdRange.push_back(thresholdedRangeandAngles.range[i]);
    }
    return msg;
}

// Function calculating the range and angels of centre for each of the locally best beams
rangeAndAngle findingRangeAndAngles(binAndIntensity bestLocalBins, int num_bins, float max_range) {
    // Necesarry variables
    rangeAndAngle rangeAndAngles;
    float angle_in_radians, range;
    
    // Calculating angles and ranges
    for (int i = 0; i < 256; i++) {
        angle_in_radians = asin((float((2*i-256))/256.0)*0.86602540); // Formula given by Tritech
        range = max_range * (float(bestLocalBins.num_bin[i]+1)/float(num_bins)); // 

        // Checking if no no bins are above threshold in that case sets range to -1
        if (bestLocalBins.intensity[i] == 0) {
            range = -1;
        }
        
        // Inserting range and angle_in_radians intro structures
        rangeAndAngles.range.push_back(range);
        rangeAndAngles.angle.push_back(angle_in_radians);
    }
    return rangeAndAngles;
}

// Function looping through all beams, extracting the first local maximum bin in each beam over a certain threshold
binAndIntensity findingBestLocalBins(MatrixXi sonarData, int num_bins, int num_beams) {
    // Necesarry variables
    binAndIntensity bestLocalBins;
    int min_intensity, threshold_intensity, num_bin; 
    bool maximum_found; 

    // Looping through each beam and finding the first local maximum bin
    min_intensity = 100;
    for (int i = 0; i < num_beams; i++) {
        num_bin = 0;
        threshold_intensity = 0;
        maximum_found = false;
        for (int j = 0; j < num_bins; j++) {
            if ((sonarData(i,j) > min_intensity) && (sonarData(i,j) > threshold_intensity)) {
                threshold_intensity = sonarData(i,j);
                num_bin = j;
                maximum_found = true;
            }
            //if ((maximum_found == true) && (sonarData(i,j) != threshold_intensity)) {
            //    j = num_bins;
            //}
        }
        //cout << "Intensity: " << threshold_intensity << ", Num_bin: " << num_bin << endl;
        bestLocalBins.intensity.push_back(threshold_intensity);
        bestLocalBins.num_bin.push_back(num_bin);
    }
    return bestLocalBins;
}

// Thresholding
binBeamAndIntensity thresholding(MatrixXi sonarData, int num_bins, int num_beams) {
    binBeamAndIntensity thresholdedBins;
    int threshold = 80;
    for (int i = 0; i < num_beams; i++) {
        for (int j = 0; j < num_bins; j++) {
            if (sonarData(i,j) > threshold) {
                thresholdedBins.intensity.push_back(sonarData(i,j));
                thresholdedBins.num_bin.push_back(j);
                thresholdedBins.num_beam.push_back(i);

            }
        }

    }
    return thresholdedBins;
}


rangeAndAngle findingRangeAndAnglesThresholding(binBeamAndIntensity bestLocalBins,int num_bins, float max_range) {
    // Necesarry variables
    rangeAndAngle rangeAndAngles;
    float angle_in_radians, range;
    
    // Calculating angles and ranges
    for (int i = 0; i < bestLocalBins.num_beam.size(); i++) {
        angle_in_radians = asin((float((2*bestLocalBins.num_beam[i]-256))/256.0)*0.86602540); // Formula given by Tritech
        range = max_range * (float(bestLocalBins.num_bin[i]+1)/float(num_bins)); // 

        // Checking if no no bins are above threshold in that case sets range to -1
        if (bestLocalBins.intensity[i] == 0) {
            range = -1;
        }
        
        // Inserting range and angle_in_radians intro structures
        rangeAndAngles.range.push_back(range);
        rangeAndAngles.angle.push_back(angle_in_radians);
    }
    return rangeAndAngles;
}


void sonarProcessor::sonarCallback(const sonar_msgs::sonar_raw_data::ConstPtr& msg) {
    // Necesarry variables
    int num_bins, num_beams, counter;
    float max_range;
    MatrixXi sonarData;
    binAndIntensity bestLocalBins;
    rangeAndAngle rangeAndAngles, thresholdedRangeandAngles;
    sonar_msgs::sonar_processed_data msgToBePublished;
    binBeamAndIntensity thresholdedStuff;
    // Extracting data from the raw data
    counter = 0;
    num_bins = msg->bins;
    num_beams = (int)(msg->data.size()/num_bins);
    max_range = msg->range;
    
    sonarData = MatrixXi(num_beams,num_bins);
    sonarData.setConstant(0);
    for (int i = 0; i < num_beams; i++) {
        for (int j = 0; j < num_bins; j++) {
            sonarData(i,j) = (int)msg->data[counter];
            counter = counter + 1;
        }
    }

    // Finding first local maximum bin in each beam
    bestLocalBins = findingBestLocalBins(sonarData, num_bins, num_beams);
   
    thresholdedStuff = thresholding(sonarData, num_bins, num_beams);
   
    // Calculate angles and ranges of bins
    thresholdedRangeandAngles = findingRangeAndAnglesThresholding(thresholdedStuff,num_bins,max_range);
    rangeAndAngles = findingRangeAndAngles(bestLocalBins, num_bins, max_range);

    // Inserting the processed data into a msg and publishing the msg
    msgToBePublished = createMsg(rangeAndAngles,thresholdedRangeandAngles);
    
    sonar_data_pub.publish(msgToBePublished);

    // Visualising function for testing purposes 
    
    //visualisingTest(sonarData, rangeAndAngles, num_bins, num_beams, max_range);


    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "sonarProcessor");
    sonarProcessor run_processor;
    ros::spin();
    return 0;
}


/* PLAN:
1. Få CMakelist.txt til og generere sonar_processed_data.msg --> Done!
2. Lag funksjonen som tar rangeAndAngles strukturen og publiserer en sonar_processed_data.msg --> Done!
3. Feilsøk ved og teste noden med baggen som er tatt opp og skriv et raskt program som visualiserer dataen --> Done!
4. Gå tilbake til sonar driveren og få inn adaptiv oppførsel for størelsen på data_table, send range i sonar_data_raw meldings typen og
ikke minst finn ut hvordan/hvorfor størelsen varrierer. --> Done!
5. Lag et sonar directory i ROS hvor sonar_driver, sonar_processor og sonar_msg ligger. --> Done!
6. Sett opp CMake filene slik at punkt 5 fungerer som det skal og alt er linket. --> Done!
7. Sett opp ROSparams som gir mening. --> Done!
8. Sett opp en launch fil som launcher som loader rosparams, launcher driveren og egentlig får alt til og fungere. --> Done!
9. Om dette tar mye mindre tid enn forventet(HAHA) se om man kan få lagt inn shared object filene på noen måte i strukturen
slik at man ikke manuelt må legge de inn i /usr/lib for det suger --> Ikke verd og prioritere og bruke tid på dette
*/
