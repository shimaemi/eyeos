/**
 * @brief Match list of 3D objects (vector<BoundingBox>) between current and previous frame
 *
 * @param matches       List of best matches between previous and current frame
 * @param bbBestMatches Output list of best matches between previous and current frame (matched boxID pairs)
 * @param prevFrame     Previous frame
 * @param currFrame     Current frame
 */
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{

    /* STEP 1 : Go through all keypoint matches and associate them with their respective bounding boxes in both images */

    std::multimap<int, int> bbTempMatches;

    // Loop over all the keypoint match pairs
    for (auto matchIt = matches.begin(); matchIt != matches.end(); ++matchIt)
    {
        int prevImgBoxID = -1;
        int currImgBoxID = -1;

        // Loop through all the BBoxes in previous image and find the box ID of the 'query' keypoint(match keypoint in previous image)
        for (auto it = prevFrame.boundingBoxes.begin(); it != prevFrame.boundingBoxes.end(); ++it)
        {
            cv::KeyPoint keyPt;
            keyPt = prevFrame.keypoints[matchIt->queryIdx];
            if (it->roi.contains(keyPt.pt))
            // if ((keyPt.pt.x > (it->roi.x)) && (keyPt.pt.x < (it->roi.x + it->roi.width)) &&
            //     (keyPt.pt.y > (it->roi.y)) && (keyPt.pt.y < (it->roi.y + it->roi.height)))
            {
                it->keypoints.push_back(keyPt);
                prevImgBoxID = it->boxID;
                break;
            }
        }

        // Loop through all the BBoxes in current image and find the box ID of the 'train' keypoint(match keypoint in current image)
        for (auto it = currFrame.boundingBoxes.begin(); it != currFrame.boundingBoxes.end(); ++it)
        {
            cv::KeyPoint keyPt;
            keyPt = currFrame.keypoints[matchIt->trainIdx];
            if (it->roi.contains(keyPt.pt))
            // if ((keyPt.pt.x > (it->roi.x)) && (keyPt.pt.x < (it->roi.x + it->roi.width)) &&
            //     (keyPt.pt.y > (it->roi.y)) && (keyPt.pt.y < (it->roi.y + it->roi.height)))
            {
                it->keypoints.push_back(keyPt);
                currImgBoxID = it->boxID;
                break;
            }
        }

        // Store the box ID pairs in a temporary multimap
        if ((prevImgBoxID != -1) && (currImgBoxID != -1)) // Exclude pairs which are not part of either BBoxes
        {
            bbTempMatches.insert(std::make_pair(prevImgBoxID, currImgBoxID));
        }

    } // eof Loop over all keypoint match pairs

    /* STEP 2: For each BBox pair count the number of keypoint matches*/

    // Find all the unique keys (BBox IDs in the prev image) from the multimap
    std::set<int> unique_keys;
    int last_key = INT_MIN; // some value that won't appear

    for (auto it = bbTempMatches.begin(); it != bbTempMatches.end(); ++it)
    {
        if (it->first != last_key)
        {
            unique_keys.insert(it->first);
            last_key = it->first;
        }
    }

    // Display contents of multimap
    if (false)
    {
        for (auto itr = bbTempMatches.begin(); itr != bbTempMatches.end(); ++itr)
        {
            cout << '\t' << itr->first << '\t' << itr->second
                 << '\n';
        }
    }

    // Create a map to count occurrences of each key-value pair
    std::map<std::pair<int, int>, int> count_map;

    // Loop through each element in the multimap
    for (auto it = bbTempMatches.begin(); it != bbTempMatches.end(); ++it)
    {
        // Create a pair from the key and value of multimap
        std::pair<int, int> key_value_pair = std::make_pair(it->first, it->second);

        // Check if the pair already exists in count_map
        if (count_map.find(key_value_pair) == count_map.end())
        {
            // If not, insert it with a count of 1
            count_map.insert(std::make_pair(key_value_pair, 1));
        }
        else
        {
            // If it exists, increment the count
            count_map[key_value_pair]++;
        }
    }

    // Display the count of each key-value pair
    if (false)
    {
        for (auto it = count_map.begin(); it != count_map.end(); ++it)
        {
            std::cout << "(" << it->first.first << ", " << it->first.second << "): " << it->second << std::endl;
        }
    }

    /* STEP 3: The BBox pair with highest number of keypoint match occurences is the best matched BBox pair*/

    // Iterate through each unique bounding box IDs in the previous image
    for (auto it = unique_keys.begin(); it != unique_keys.end(); ++it)
    {
        int BBoxIdx1 = -1; // BBox index
        int BBoxIdx2 = -1;
        int maxKyPtCnt = INT_MIN;

        // Loop through all the BBox matched pairs and find the ones with highest keypoint occurences
        for (auto it1 = count_map.begin(); it1 != count_map.end(); ++it1)
        {
            int currKyPtCnt = it1->second; // Number of occurences for the current BBox pair

            if (it1->first.first == *it)
            {
                if (currKyPtCnt >= maxKyPtCnt)
                {
                    maxKyPtCnt = currKyPtCnt;
                    BBoxIdx1 = it1->first.first;
                    BBoxIdx2 = it1->first.second;
                }
            }
        }

        if ((BBoxIdx1 != -1) && (BBoxIdx2 != -1)) // Exclude pairs which are not part of either BBoxes
        {
            bbBestMatches.insert(std::make_pair(BBoxIdx1, BBoxIdx2));
        }
    }

    // Display the count of each BBox pair
    if (false)
    {
        for (auto it = bbBestMatches.begin(); it != bbBestMatches.end(); ++it)
        {
            std::cout << "(" << it->first << ", " << it->second << "): " << std::endl;
        }
    }
}


/**
 * @brief Cluster keypoint matches with the current bounding box
 *
 * @param boundingBox   Current bounding box
 * @param kptsPrev      Previous frame keypoints
 * @param kptsCurr      Current frame keypoints
 * @param kptMatches    Keypoint matches between previous and current frame
 */
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    /* STEP 1: Associate the keypoint match with the bounding box and calculate keypoint match distance*/

    // Loop through all the keypoint matches and find the ones which are part of the current bounding box
    // Also find the distance between the matched keypoints
    std::vector<double> distKptMatches;
    for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
        cv::KeyPoint keyPtPrev, keyPtCurr;
        double dist;
        keyPtPrev = kptsPrev[it->queryIdx];
        keyPtCurr = kptsCurr[it->trainIdx];

        if (boundingBox.roi.contains(keyPtCurr.pt))
        {
            boundingBox.kptMatches.push_back(*it);
            dist = cv::norm(keyPtCurr.pt - keyPtPrev.pt);
            distKptMatches.push_back(dist);
        }
    }

    /*STEP 2: Remove outlier keypoint matches from the bounding box*/

    // Get the Q1 and Q3 percentile for the distance vector
    double q1 = percentile(distKptMatches, 0.25);
    double q3 = percentile(distKptMatches, 0.75);
    // Find the IQR for the distance vector
    double iqr = q3 - q1;

    // Go through all the matched keypoint pairs in the current bounding box and remove the ones which are outliers from the bounding box
    auto it1 = boundingBox.kptMatches.begin();
    while (it1 != boundingBox.kptMatches.end())
    {
        cv::KeyPoint keyPtPrev, keyPtCurr;
        double dist;
        keyPtPrev = kptsPrev[it1->queryIdx];
        keyPtCurr = kptsCurr[it1->trainIdx];
        dist = cv::norm(keyPtCurr.pt - keyPtPrev.pt);

        if ((dist < (q1 - 1.5 * iqr)) || (dist > (q3 + 1.5 * iqr)))
        {
            it1 = boundingBox.kptMatches.erase(it1); // erase() returns the next iterator
        }
        else
        {
            ++it1;
        }
    }
}


/**
 * @brief Compute time-to-collision (TTC) based on keypoint correspondences in successive images
 * 
 * @param kptsPrev      Previous frame keypoints
 * @param kptsCurr      Current frame keypoints
 * @param kptMatches    Keypoint matches between previous and current frame
 * @param frameRate     Frame rate of the camera 
 * @param TTC           Output TTC
 * @param visImg        Output visualization image
 */
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    /* STEP 1: Compute distance ratios between all matched keypoints*/

    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint match  loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint match loop

            double minDist = 100.0; // min. required distance in pixels

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    /* STEP 2: Filter out outliers from the distance ratio vector*/

    // Remove the outliers from the distance ratio vector
    std::vector<double> filtDistRatios = removeOutliers(distRatios);

    /* STEP 3: Compute camera-based TTC from distance ratios*/

    // compute camera-based TTC from mean distance ratios
    double meanDistRatio = std::accumulate(filtDistRatios.begin(), filtDistRatios.end(), 0.0) / filtDistRatios.size();

    double dT = 1 / frameRate;
    // TTC = -dT / (1 - meanDistRatio);

    // Alternate method to compute camera-based TTC from distance ratios using median
    double medianDistRatio;

    std::sort(filtDistRatios.begin(), filtDistRatios.end());
    if (distRatios.size() % 2 == 0)
    {
        medianDistRatio = (filtDistRatios[filtDistRatios.size() / 2 - 1] + filtDistRatios[filtDistRatios.size() / 2]) / 2;
    }
    else
    {
        medianDistRatio = filtDistRatios[filtDistRatios.size() / 2];
    }

    TTC = -dT / (1 - medianDistRatio);
}