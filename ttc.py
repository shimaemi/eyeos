def match_bounding_boxes(matches, bb_best_matches, prev_frame, curr_frame):

    # STEP 1: Go through all keypoint matches and associate them with their respective bounding boxes in both images
    bb_temp_matches = defaultdict(list)

    # Loop over all the keypoint match pairs
    for match in matches:
        prev_img_box_id = -1
        curr_img_box_id = -1

        # Loop through all the BBoxes in previous image
        for bbox in prev_frame.bounding_boxes:
            key_pt = prev_frame.keypoints[match.queryIdx]
            if bbox.roi.contains(key_pt.pt):
                bbox.keypoints.append(key_pt)
                prev_img_box_id = bbox.boxID
                break

        # Loop through all the BBoxes in current image
        for bbox in curr_frame.bounding_boxes:
            key_pt = curr_frame.keypoints[match.trainIdx]
            if bbox.roi.contains(key_pt.pt):
                bbox.keypoints.append(key_pt)
                curr_img_box_id = bbox.boxID
                break

        # Store the box ID pairs in a temporary dictionary
        if prev_img_box_id != -1 and curr_img_box_id != -1:  # Exclude pairs which are not part of either BBoxes
            bb_temp_matches[prev_img_box_id].append(curr_img_box_id)

    # STEP 2: For each BBox pair count the number of keypoint matches
    unique_keys = set()
    last_key = float('-inf')  # some value that won't appear

    for key in bb_temp_matches.keys():
        if key != last_key:
            unique_keys.add(key)
            last_key = key

    # Display contents of multimap (if needed)
    if False:
        for key, value in bb_temp_matches.items():
            print(f'\t{key}\t{value}')

    # Create a map to count occurrences of each key-value pair
    count_map = defaultdict(int)

    # Loop through each element in the multimap
    for key, value in bbTempMatches.items():
        # Create a pair from the key and value of multimap
        key_value_pair = (key, value)

        # Increment the count for the pair
        count_map[key_value_pair] += 1

    # Display the count of each key-value pair
    if False:
        for key_value_pair, count in count_map.items():
            print(f"({key_value_pair[0]}, {key_value_pair[1]}): {count}")

    # STEP 3: The BBox pair with highest number of keypoint match occurrences is the best matched BBox pair

    # Iterate through each unique bounding box IDs in the previous image
    for unique_key in unique_keys:
        BBoxIdx1 = -1  # BBox index
        BBoxIdx2 = -1
        maxKyPtCnt = float('-inf')

        # Loop through all the BBox matched pairs and find the ones with highest keypoint occurrences
        for (key1, key2), currKyPtCnt in count_map.items():
            if key1 == unique_key:
                if currKyPtCnt >= maxKyPtCnt:
                    maxKyPtCnt = currKyPtCnt
                    BBoxIdx1 = key1
                    BBoxIdx2 = key2

        if (BBoxIdx1 != -1) and (BBoxIdx2 != -1):  # Exclude pairs which are not part of either BBoxes
            bbBestMatches.add((BBoxIdx1, BBoxIdx2))

    # Display the count of each BBox pair
    if False:
        for BBox_pair in bbBestMatches:
            print(f"({BBox_pair[0]}, {BBox_pair[1]}): ")


def cluster_kpt_matches_with_roi(bounding_box, kpts_prev, kpts_curr, kpt_matches):
    # STEP 1: Associate the keypoint match with the bounding box and calculate keypoint match distance

    # Loop through all the keypoint matches and find the ones which are part of the current bounding box
    # Also find the distance between the matched keypoints
    dist_kpt_matches = []
    for match in kpt_matches:
        key_pt_prev = kpts_prev[match.queryIdx]
        key_pt_curr = kpts_curr[match.trainIdx]

        if bounding_box.roi.contains(key_pt_curr.pt):
            bounding_box.kptMatches.append(match)
            dist = np.linalg.norm(key_pt_curr.pt - key_pt_prev.pt)
            dist_kpt_matches.append(dist)

    # STEP 2: Remove outlier keypoint matches from the bounding box

    # Get the Q1 and Q3 percentile for the distance vector
    q1 = np.percentile(dist_kpt_matches, 25)
    q3 = np.percentile(dist_kpt_matches, 75)
    # Find the IQR for the distance vector
    iqr = q3 - q1

    # Go through all the matched keypoint pairs in the current bounding box and remove the ones which are outliers from the bounding box
    it1 = 0
    while it1 < len(bounding_box.kptMatches):
        match = bounding_box.kptMatches[it1]
        key_pt_prev = kpts_prev[match.queryIdx]
        key_pt_curr = kpts_curr[match.trainIdx]
        dist = np.linalg.norm(key_pt_curr.pt - key_pt_prev.pt)

        if (dist < (q1 - 1.5 * iqr)) or (dist > (q3 + 1.5 * iqr)):
            bounding_box.kptMatches.pop(it1)  # remove the outlier
        else:
            it1 += 1

def computeTTCCamera(kptsPrev, kptsCurr, kptMatches, frameRate):
    distRatios = []  # stores the distance ratios for all keypoints between curr. and prev. frame

    for i in range(len(kptMatches) - 1):  # outer keypoint match loop
        kpOuterCurr = kptsCurr[kptMatches[i].trainIdx]
        kpOuterPrev = kptsPrev[kptMatches[i].queryIdx]

        for j in range(1, len(kptMatches)):  # inner keypoint match loop
            minDist = 100.0  # min. required distance in pixels
            kpInnerCurr = kptsCurr[kptMatches[j].trainIdx]
            kpInnerPrev = kptsPrev[kptMatches[j].queryIdx]

            distCurr = np.linalg.norm(np.array(kpOuterCurr.pt) - np.array(kpInnerCurr.pt))
            distPrev = np.linalg.norm(np.array(kpOuterPrev.pt) - np.array(kpInnerPrev.pt))

            if distPrev > np.finfo(float).eps and distCurr >= minDist:  # avoid division by zero
                distRatio = distCurr / distPrev
                distRatios.append(distRatio)

    # only continue if list of distance ratios is not empty
    if len(distRatios) == 0:
        return float('nan')

    dT = 1 / frameRate

    TTC = -dT / (1 - np.mean(distRatios))
    return TTC