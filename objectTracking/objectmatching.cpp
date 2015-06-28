#include "objectmatching.h"

ObjectMatching::ObjectMatching()
{
    hist_size = 255;
    hist_ranges = new float[]({1, 360});
    fg_om_existed = false;
    fg_om_prev_existed = false;

#ifdef debug_info_object_matching_img
    cv::namedWindow("Comp", cv::WINDOW_AUTOSIZE);
#endif
}

ObjectMatching::~ObjectMatching()
{
    if (fg_initialized) {
        delete[] om;
        delete[] om_tmp;
        delete[] om_prev;
    }
}

void ObjectMatching::initializeObjectMatching(int size)
{
    if (fg_initialized) {
        if (size != om_size || size != om_prev_size) {
            delete[] om;
            delete[] om_tmp;
            delete[] om_prev;
            fg_initialized = false;
        }
    }
    om = new objectMatchingInfo[size];
    om_tmp = new objectMatchingInfo[size];
    om_prev = new objectMatchingInfo[size];
    om_size = size;
    om_prev_size = size;
    for (int i = 0; i < om_size; i++)
        resetMatchingInfo(om[i]);
    for (int i = 0; i < om_prev_size; i++)
        resetMatchingInfo(om_prev[i]);

    fg_initialized = true;
}

void ObjectMatching::setThresholdBha(double thresh)
{
    thresh_Bha = thresh;
}

void ObjectMatching::setErrorThresholdZ(double max_err_z, double max_distance_z)
{
    this->max_err_z = max_err_z;
    this->max_distance_z = max_distance_z;
}

void ObjectMatching::setErrorThresholdX(double max_err_x)
{
    this->max_err_x = max_err_x;
}

void ObjectMatching::splitOneOut(int channel, cv::Mat src, cv::Mat *dst)
{
    if (src.size != dst->size)
        dst->release();
    if (dst->empty())
        dst->create(src.rows, src.cols, CV_8UC1);
    int tc = src.channels();
    for (int r = 0; r < src.rows; r++) {
        uchar *ptr = src.ptr<uchar>(r);
        for (int c = 0; c < src.cols; c++) {
            dst->ptr<uchar>(r)[c] = ptr[tc * c + channel];
        }
    }
}

void ObjectMatching::resetMatchingInfo(objectMatchingInfo &src)
{
    src.pc.range = 0.0;
    src.pc.angle = 0.0;
    src.img.release();
    src.H_img.release();
    src.H_hist.release();
    src.fg_Bha_check = false;
    src.labeled = false;
    src.match_type = -1;
}

void ObjectMatching::moveOm(objectMatchingInfo &src, objectMatchingInfo &dst)
{
    dst.labeled        = src.labeled;
    dst.match_type     = src.match_type;
    dst.pc.range       = src.pc.range;
    dst.pc.angle       = src.pc.angle;
    dst.err_pos.first  = src.err_pos.first;
    dst.err_pos.second = src.err_pos.second;
    dst.img.release();
    dst.H_img.release();
    dst.H_hist.release();
    dst.img            = src.img.clone();
    dst.H_img          = src.H_img.clone();
    dst.H_hist         = src.H_hist.clone();
    dst.fg_Bha_check   = src.fg_Bha_check;
    dst.center.first   = src.center.first;
    dst.center.second  = src.center.second;

    resetMatchingInfo(*&src);
}

void ObjectMatching::resetObjMatching()
{
    matchingList.clear();
    fg_om_prev_existed = fg_om_existed;
    fg_om_existed = false;
    map_Bha_corr_id_r.clear();
    map_Bha_corr_id_c.clear();
    map_Bha.release();
    map_err_pos_x.release();
    map_err_pos_z.release();
    map_match_type.release();
    map_thresh_err_x.release();
    map_thresh_err_z.release();
    for (int i = 0; i < om_size; i++)
        resetMatchingInfo(om[i]);
    om_obj_num = 0;
}

std::vector<ObjectMatching::matchedResult> ObjectMatching::Matching()
{
    if (!fg_initialized)
        return matchingList;
    // Object matching: a) Bha. dist. of H color space, b) bias of X & Z of WCS location
    // Comparison of H color space image using Bhattacharyya distance with Bubble search
    // record detected object's id
    for (int m = 0; m < om_prev_size; m++)
        if (om_prev[m].labeled) {
            map_Bha_corr_id_r.push_back(m);
        }
    for (int m = 0; m < om_size; m++)
        if (om[m].labeled) {
            map_Bha_corr_id_c.push_back(m);
        }
    om_prev_obj_num = map_Bha_corr_id_r.size();
    om_obj_num = map_Bha_corr_id_c.size();
#ifdef debug_info_object_matching_others
        std::cout<<"===============\n";
        std::cout<<"size: om "<<om_obj_num<<" om_prev "<<om_prev_obj_num<<std::endl;
#endif

    // Bhattacharyya distance
    if (fg_om_existed && fg_om_prev_existed) {
        map_Bha = cv::Mat(om_prev_obj_num, om_obj_num, CV_64F, cv::Scalar(1.1));
        map_err_pos_x = cv::Mat(om_prev_obj_num, om_obj_num, CV_64F, cv::Scalar(1.1));
        map_err_pos_z = cv::Mat(om_prev_obj_num, om_obj_num, CV_64F, cv::Scalar(1.1));
        map_match_type = cv::Mat(om_prev_obj_num, om_obj_num, CV_32S, cv::Scalar(-1));
        map_thresh_err_x = cv::Mat(om_prev_obj_num, 1, CV_64F, cv::Scalar(0.0));
        map_thresh_err_z = cv::Mat(om_obj_num, 1, CV_64F, cv::Scalar(0.0));
        for (int m = 0; m < map_Bha_corr_id_r.size(); m++) {
            for (int n = 0; n < map_Bha_corr_id_c.size(); n++) {
                int id, id_prev;
                id_prev = map_Bha_corr_id_r[m];
                id = map_Bha_corr_id_c[n];
                // H color space object image comparison
                if (om_prev[id_prev].match_type == MATCH_TYPE::RANGE_BHA && om[id].match_type == MATCH_TYPE::RANGE_BHA) {
                    map_Bha.ptr<double>(m)[n] = cv::compareHist(om_prev[id_prev].H_hist, om[id].H_hist, cv::HISTCMP_BHATTACHARYYA);
                    map_match_type.ptr<int>(m)[n] = MATCH_TYPE::RANGE_BHA;
                }
                else
                    map_match_type.ptr<int>(m)[n] = MATCH_TYPE::RANGE_ONLY;

                // WCS location comparison (X, Z)
                std::pair<double, double> pc, pc_prev;
                pc_prev.first = om_prev[id_prev].pc.range * sin(om_prev[id_prev].pc.angle * CV_PI / 180.0);
                pc_prev.second = om_prev[id_prev].pc.range * cos(om_prev[id_prev].pc.angle * CV_PI / 180.0);
                pc.first = om[id].pc.range * sin(om[id].pc.angle * CV_PI / 180.0);
                pc.second = om[id].pc.range * cos(om[id].pc.angle * CV_PI / 180.0);
                map_err_pos_x.ptr<double>(m)[n] = fabs((pc.first - pc_prev.first));
                map_err_pos_z.ptr<double>(m)[n] = fabs((pc.second - pc_prev.second));

                double ratio = pc.second / max_distance_z;
                if (m == 0)
                    map_thresh_err_z.ptr<double>(n)[0] = max_err_z * ratio;
                if (n == 0)
                    map_thresh_err_x.ptr<double>(m)[0] = max_err_x;
            }
        }

#ifdef debug_info_object_matching_others
        std::cout<<"Bha map\n";
        for (int m = 0; m < map_Bha_corr_id_r.size(); m++) {
            for (int n = 0; n < map_Bha_corr_id_c.size(); n++) {
                std::cout<<map_Bha.ptr<double>(m)[n]<<"\t\t";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;

        std::cout<<"Err pos X-Z map\n";
        for (int m = 0; m < map_Bha_corr_id_r.size(); m++) {
            for (int n = 0; n < map_Bha_corr_id_c.size(); n++) {
                int id, id_prev;
                id_prev = map_Bha_corr_id_r[m];
                id = map_Bha_corr_id_c[n];
                std::cout<<map_err_pos_x.ptr<double>(m)[n]<<" "<<map_err_pos_z.ptr<double>(m)[n]<<"\t\t\t";
            }
            std::cout<<std::endl;
        }
        std::cout<<std::endl;
#endif

        std::vector<cv::Point> sort_min;    // contains correspondence between prev. and now on map. (prev, now)
        double Bha_min, range_min;
        cv::Point min_count = cv::Point(-1, -1);
        cv::Mat_<bool> check_map_Bha_r = cv::Mat_<bool>(om_prev_obj_num, 1, false);
        cv::Mat_<bool> check_map_Bha_c = cv::Mat_<bool>(om_obj_num, 1, false);
        for (int p = 0; p < om_prev_obj_num; p++) {
            Bha_min = 2.0;   // The max. of Bha. Dist. is 1.0, so greater than 1.0 is ok.
            range_min = 1000000000.0;
            min_count.x = -1;
            min_count.y = -1;
            for (int r = 0; r < om_prev_obj_num; r++) {
                for (int c = 0; c < om_obj_num; c++) {
                    if (check_map_Bha_r.at<bool>(r, 0) == true || check_map_Bha_c.at<bool>(c, 0) == true)
                        continue;
                    //**// better solution? range error can't be normalized.
                    double val = sqrt(pow(map_err_pos_x.ptr<double>(r)[c], 2) + pow(map_err_pos_z.ptr<double>(r)[c], 2));
                    if (map_match_type.ptr<int>(r)[c] == MATCH_TYPE::RANGE_BHA) {
                        if (Bha_min > map_Bha.ptr<double>(r)[c] || range_min > val) {
                            Bha_min = map_Bha.ptr<double>(r)[c];
                            range_min = val;
                            min_count.x = r;
                            min_count.y = c;
                        }
                    }
                    else if (map_match_type.ptr<int>(r)[c] == MATCH_TYPE::RANGE_ONLY) {
                        if (range_min > val) {
                            range_min = val;
                            min_count.x = r;
                            min_count.y = c;
                        }
                    }
                }
            }
            if (min_count.x == -1 && min_count.x == -1)
                continue;
            // Check if the object satisfies the threshold
            // [RANGE_BHA] if Bha. dist. and range are less than specific thresholds, it's probably a successful match.
            // [RANGE] if range is less than threshold, it's probably a sucessful match.
            if ((map_match_type.ptr<int>(min_count.x)[min_count.y] == MATCH_TYPE::RANGE_BHA && map_Bha.ptr<double>(min_count.x)[min_count.y] <= thresh_Bha ||
                 map_match_type.ptr<int>(min_count.x)[min_count.y] == MATCH_TYPE::RANGE_ONLY) &&
                    map_err_pos_x.ptr<double>(min_count.x)[min_count.y] <= map_thresh_err_x.ptr<double>(min_count.x)[0] &&
                    map_err_pos_z.ptr<double>(min_count.x)[min_count.y] <= map_thresh_err_z.ptr<double>(min_count.y)[0]) {
                sort_min.push_back(min_count);
            }
            check_map_Bha_r.at<bool>(min_count.x) = true;
            check_map_Bha_c.at<bool>(min_count.y) = true;
        }

        // store corresponding matching result, id is followed by previous id
#ifdef debug_info_object_matching_others
        std::cout<<"==============="<<std::endl;
#endif
        for (int i = 0; i < map_Bha_corr_id_c.size(); i++) {
            int id = map_Bha_corr_id_c[i];
            int prev_id = -1;
            for (int j = 0; j < sort_min.size(); j++) {
                if (map_Bha_corr_id_c[sort_min[j].y] == id) {
                    prev_id = map_Bha_corr_id_r[sort_min[j].x];
#ifdef debug_info_object_matching_others
                    std::cout<<"matched "<<map_Bha_corr_id_c[sort_min[j].y]<<" "<<id<<
                              " range "<<om[id].pc.range<<" previous id "<<map_Bha_corr_id_r[sort_min[j].x]<<std::endl;
#endif
                    break;
                }
            }
            matchingList.push_back(matchedResult(id, prev_id));
        }
#ifdef debug_info_object_matching_others
        std::cout<<"==============="<<std::endl;
#endif

#ifdef debug_info_object_matching_others
        // SV check =============
        std::cout<<"\nmatched\n";
        for (int i = 0; i < sort_min.size(); i++) {
            int id = map_Bha_corr_id_c[sort_min[i].y];
            int id_prev = map_Bha_corr_id_r[sort_min[i].x];
            std::cout<<"prev "<<id_prev<<", now "<<id<<std::endl;
        }

        std::cout<<"OM============\nBefore: \n";
        for (int i = 0; i < om_prev_size; i++) {
            if (om_prev[i].labeled) {
                std::cout<<"label: "<<i<<std::endl;
            }
        }
        std::cout<<"After: \n";
        for (int i = 0; i < om_size; i++) {
            if (om[i].labeled) {
                std::cout<<"label: "<<i<<std::endl;
            }
        }
        // ======================

        for (int i = 0 ; i < sort_min.size(); i++) {
            std::cout<<"prev "<<sort_min[i].x<<" ("<<map_Bha_corr_id_r[sort_min[i].x]<<
                       "), now "<<sort_min[i].y<<" ("<<map_Bha_corr_id_c[sort_min[i].y]<<")\t";
#ifdef debug_info_sv_object_matching_data_extract
            cv::imshow("Image - now", om[map_Bha_corr_id_c[sort_min[i].y]].img);
            cv::imshow("Image - prev", om_prev[map_Bha_corr_id_r[sort_min[i].x]].img);
            char c = cv::waitKey();
            if (c == '1')
                std::cout<<"correct "<<map_Bha.ptr<double>(sort_min[i].x)[sort_min[i].y]<<std::endl;
            else if (c == '2')
                std::cout<<"wrong "<<map_Bha.ptr<double>(sort_min[i].x)[sort_min[i].y]<<std::endl;
#endif
        }
        std::cout<<std::endl;
#endif
#ifdef debug_info_object_matching_img
        cv::Mat comp_mix;
        if (!comp_prev.empty()) {
            comp_mix = cv::Mat::zeros(2 * comp.rows, comp.cols, CV_8UC3);
            comp_prev.copyTo(comp_mix(cv::Rect(0, 0, comp.cols, comp.rows)));
            comp.copyTo(comp_mix(cv::Rect(0, comp.rows, comp.cols, comp.rows)));
            cv::putText(comp_mix, "Previous", cv::Point(0, 0.1 * comp.rows), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0, 255));
            cv::putText(comp_mix, "Now", cv::Point(0, 1.1 * comp.rows), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0, 255));
            cv::line(comp_mix, cv::Point(0, comp.rows), cv::Point(comp.cols, comp.rows), cv::Scalar(255, 255, 255), 1, 8, 0);
        }

        // draw matched result of previous and now
        for (int i = 0; i < om_prev_size; i++) {
            if (om_prev[i].labeled) {
                cv::Point p1 = cv::Point(om_prev[i].center.second, om_prev[i].center.first );
                cv::putText(comp_mix, QString::number(i).toStdString(), p1, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));
            }
        }
        for (int i = 0; i < om_size; i++) {
            if (om[i].labeled) {
                cv::Point p2 = cv::Point(om[i].center.second, om[i].center.first + comp.rows);
                cv::putText(comp_mix, QString::number(i).toStdString(), p2, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0));
            }
        }
        for (int i = 0 ; i < sort_min.size(); i++) {
            if (!comp_prev.empty()) {
                cv::Point p1, p2;
                p1 = cv::Point(om_prev[map_Bha_corr_id_r[sort_min[i].x]].center.second, om_prev[map_Bha_corr_id_r[sort_min[i].x]].center.first);
                p2 = cv::Point(om[map_Bha_corr_id_c[sort_min[i].y]].center.second, om[map_Bha_corr_id_c[sort_min[i].y]].center.first + comp.rows);
                cv::line(comp_mix, p1, p2, cv::Scalar(255, 255, 255), 2, 8, 0);
            }
        }
#ifdef debug_info_object_matching_others
        std::cout<<"Pos X-Z map\n";
        for (int i = 0 ; i < sort_min.size(); i++) {
            if (!comp_prev.empty()) {
                cv::Point p1, p2;
                p1 = cv::Point(om_prev[map_Bha_corr_id_r[sort_min[i].x]].center.second, om_prev[map_Bha_corr_id_r[sort_min[i].x]].center.first);
                p2 = cv::Point(om[map_Bha_corr_id_c[sort_min[i].y]].center.second, om[map_Bha_corr_id_c[sort_min[i].y]].center.first + comp.rows);
                std::cout<<p1.x<<", "<<p1.y<<" <-> "<<p2.x<<", "<<p2.y<<"\t";
            }
        }
        std::cout<<std::endl;
#endif

        if (!comp_prev.empty()) {
            cv::imshow("Comp", comp_mix);
        }
        comp_prev.release();
        comp_prev = comp.clone();
        comp.release();
#endif
    }

    // push om to om_prev
    om_prev_obj_num = om_obj_num;
    for (int i = 0; i < om_size; i++) {
        moveOm(om[i], om_prev[i]);
    }

    return matchingList;
}
