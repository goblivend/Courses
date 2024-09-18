//
// Created by Thomas Michelot on 1/4/21.
//

#include "Export.hh"

bool Export::exportCSV(const std::string &path) {
    std::fstream outStream;

    outStream.open(path, std::fstream::out);
    if (outStream.fail() || !outStream.good() || outStream.bad()) {
        return Error;
    }

    auto writer = csv::make_csv_writer(outStream);
    static const auto header = std::vector<std::string>(
            {"Image ID",
             "Line",
             "Column",
             "Line rectified",
             "Column rectified",
             "Latitude",
             "Longitude",
             "Speed (km/h)",
             "Orientation (degree)"});

    writer << header;

    std::vector<std::string> line1;
    auto &firstImage = GeoTaggedImageList::instance().geoTaggedImageList_[0];

    line1.push_back(std::to_string(firstImage.index_ + 1)); // Image ID
    line1.push_back(std::to_string(firstImage.coordinateXY_.getY())); // Line
    line1.push_back(std::to_string(firstImage.coordinateXY_.getX())); // Column
    line1.push_back(std::to_string((int)(firstImage.coordinateXY_.getY()+Convert::LINE_OFFSET))); // Line
    line1.push_back(std::to_string((int)(firstImage.coordinateXY_.getX()+Convert::COLUMN_OFFSET))); // Column
    line1.push_back(std::to_string(Convert::getLatitude(firstImage)));
    line1.push_back(std::to_string(Convert::getLongitude(firstImage)));
	line1.emplace_back("N/A");
    line1.emplace_back("N/A");
	
    writer << line1;


    int i;
    for (i = 1; i < GeoTaggedImageList::instance().getListLength() - 1; i++) {
        auto line = taggedImageToStringVector(
                GeoTaggedImageList::instance().geoTaggedImageList_[i-1],
        		GeoTaggedImageList::instance().geoTaggedImageList_[i],
                GeoTaggedImageList::instance().geoTaggedImageList_[i + 1]
        );
        writer << line;
    }

    auto &lastImage = GeoTaggedImageList::instance().geoTaggedImageList_[i];
	std::vector<std::string> line2;

    line2.push_back(std::to_string(lastImage.index_ + 1)); // Image ID
    line2.push_back(std::to_string(lastImage.coordinateXY_.getY())); // Line
    line2.push_back(std::to_string(lastImage.coordinateXY_.getX())); // Column
    line2.push_back(std::to_string((int)(lastImage.coordinateXY_.getY()+Convert::LINE_OFFSET))); // Line
    line2.push_back(std::to_string((int)(lastImage.coordinateXY_.getX()+Convert::COLUMN_OFFSET))); // Column
    line2.push_back(std::to_string(Convert::getLatitude(lastImage)));
    line2.push_back(std::to_string(Convert::getLongitude(lastImage)));
	line2.emplace_back("N/A");
    line2.emplace_back("N/A");

    writer << line2;
    return Success;
}

std::vector<std::string> Export::taggedImageToStringVector(const GeoTaggedImage &image0, const GeoTaggedImage &image1, const GeoTaggedImage &image2) {
    std::vector<std::string> line;

    line.push_back(std::to_string(image1.index_ + 1)); // Image ID
    line.push_back(std::to_string(image1.coordinateXY_.getY())); // Line
    line.push_back(std::to_string(image1.coordinateXY_.getX())); // Column

    line.push_back(std::to_string((int)(image1.coordinateXY_.getY()+Convert::LINE_OFFSET))); // Line
    line.push_back(std::to_string((int)(image1.coordinateXY_.getX()+Convert::COLUMN_OFFSET))); // Column

    line.push_back(std::to_string(Convert::getLatitude(image1)));
    line.push_back(std::to_string(Convert::getLongitude(image1)));
	
    line.push_back(std::to_string(Convert::getSpeed(image0, image2)));
    line.push_back(std::to_string(Convert::getOrientation(image0, image2)));

    return line;
}

bool Export::exportImage(const QString& path, const QPixmap &pixmap) {
    return pixmap.save(path, "PNG", 100);
}
