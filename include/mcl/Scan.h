#ifndef SCAN_H__
#define SCAN_H__

class Scan
{
public: 
	int seq_;
	int processed_seq_;
	double angle_max_;
	double angle_min_;
	double angle_increment_;
	std::vector<double> ranges_;
};

#endif
