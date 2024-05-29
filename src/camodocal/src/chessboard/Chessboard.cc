#include "camodocal/chessboard/Chessboard.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camodocal/chessboard/ChessboardQuad.h"
#include "camodocal/chessboard/Spline.h"

#define MAX_CONTOUR_APPROX  7

namespace camodocal
{

Chessboard::Chessboard(cv::Size boardSize, cv::Mat& image)
 : mBoardSize(boardSize)
 , mCornersFound(false)
{
    if (image.channels() == 1)
    {
        cv::cvtColor(image, mSketch, cv::COLOR_GRAY2BGR);
        image.copyTo(mImage);
    }
    else
    {
        image.copyTo(mSketch);
        cv::cvtColor(image, mImage, cv::COLOR_BGR2GRAY);
    }
}

void
Chessboard::findCorners(bool useOpenCV)
{
    mCornersFound = findChessboardCorners(mImage, mBoardSize, mCorners,
                                          cv::CALIB_CB_ADAPTIVE_THRESH +
                                          cv::CALIB_CB_NORMALIZE_IMAGE +
                                          cv::CALIB_CB_FILTER_QUADS +
                                          cv::CALIB_CB_FAST_CHECK,
                                          useOpenCV);

    if (mCornersFound)
    {
        // draw chessboard corners
        cv::drawChessboardCorners(mSketch, mBoardSize, mCorners, mCornersFound);
    }
}

const std::vector<cv::Point2f>&
Chessboard::getCorners(void) const
{
    return mCorners;
}

bool
Chessboard::cornersFound(void) const
{
    return mCornersFound;
}

const cv::Mat&
Chessboard::getImage(void) const
{
    return mImage;
}

const cv::Mat&
Chessboard::getSketch(void) const
{
    return mSketch;
}

bool
Chessboard::findChessboardCorners(const cv::Mat& image,
                                  const cv::Size& patternSize,
                                  std::vector<cv::Point2f>& corners,
                                  int flags, bool useOpenCV)
{
    if (useOpenCV)
    {
        return cv::findChessboardCorners(image, patternSize, corners, flags);
    }
    else
    {
        return findChessboardCornersImproved(image, patternSize, corners, flags);
    }
}

bool
Chessboard::findChessboardCornersImproved(const cv::Mat& image,
                                          const cv::Size& patternSize,
                                          std::vector<cv::Point2f>& corners,
                                          int flags)
{
    /************************************************************************************\
        This is improved variant of chessboard corner detection algorithm that
        uses a graph of connected quads. It is based on the code contributed
        by Vladimir Vezhnevets and Philip Gruebele.
        Here is the copyright notice from the original Vladimir's code:
        ===============================================================

        The algorithms developed and implemented by Vezhnevets Vldimir
        aka Dead Moroz (vvp@graphics.cs.msu.ru)
        See http://graphics.cs.msu.su/en/research/calibration/opencv.html
        for detailed information.

        Reliability additions and modifications made by Philip Gruebele.
        <a href="mailto:pgruebele@cox.net">pgruebele@cox.net</a>

        Some improvements were made by:
        1) Martin Rufli - increased chance of correct corner matching
           Rufli, M., Scaramuzza, D., and Siegwart, R. (2008),
           Automatic Detection of Checkerboards on Blurred and Distorted Images,
           Proceedings of the IEEE/RSJ International Conference on
           Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.
        2) Lionel Heng - post-detection checks and better thresholding

    \************************************************************************************/

    //int bestDilation        = -1;
    const int minDilations    =  0;
    const int maxDilations    =  7;

    std::vector<ChessboardQuadPtr> outputQuadGroup;

    if (image.depth() != CV_8U || image.channels() == 2)
    {
        return false;
    }

    if (patternSize.width < 2 || patternSize.height < 2)
    {
        return false;
    }

    if (patternSize.width > 127 || patternSize.height > 127)
    {
        return false;
    }

    cv::Mat img = image;

    // Image histogram normalization and
    // BGR to Grayscale image conversion (if applicable)
    // MARTIN: Set to "false"
    if (image.channels() != 1 || (flags & cv::CALIB_CB_NORMALIZE_IMAGE))
    {
        cv::Mat norm_img(image.rows, image.cols, CV_8UC1);

        if (image.channels() != 1)
        {
            cv::cvtColor(image, norm_img, cv::COLOR_BGR2GRAY);
            img = norm_img;
        }

        if (flags & cv::CALIB_CB_NORMALIZE_IMAGE)
        {
            cv::equalizeHist(image, norm_img);
            img = norm_img;
        }
    }

    if (flags & cv::CALIB_CB_FAST_CHECK)
    {
        if (!checkChessboard(img, patternSize))
        {
            return false;
        }
    }

    // PART 1: FIND LARGEST PATTERN
    //-----------------------------------------------------------------------
    // Checker patterns are tried to be found by dilating the background and
    // then applying a canny edge finder on the closed contours (checkers).
    // Try one dilation run, but if the pattern is not found, repeat until
    // max_dilations is reached.

    int prevSqrSize = 0;
    bool found = false;
    std::vector<ChessboardCornerPtr> outputCorners;

    for (int k = 0; k < 6; ++k)
    {
        for (int dilations = minDilations; dilations <= maxDilations; ++dilations)
        {
            if (found)
            {
                break;
            }

            cv::Mat thresh_img;

            // convert the input grayscale image to binary (black-n-white)
            if (flags & cv::CALIB_CB_ADAPTIVE_THRESH)
            {
                int blockSize = lround(prevSqrSize == 0 ?
                    std::min(img.cols,img.rows)*(k%2 == 0 ? 0.2 : 0.1): prevSqrSize*2)|1;

                // convert to binary
                cv::adaptiveThreshold(img, thresh_img, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, blockSize, (k/2)*5);
            }
            else
            {
                // empiric threshold level
                double mean = (cv::mean(img))[0];
                int thresh_level = lround(mean - 10);
                thresh_level = std::max(thresh_level, 10);

                cv::threshold(img, thresh_img, thresh_level, 255, cv::THRESH_BINARY);
            }

            // MARTIN's Code
            // Use both a rectangular and a cross kernel. In this way, a more
            // homogeneous dilation is performed, which is crucial for small,
            // distorted checkers. Use the CROSS kernel first, since its action
            // on the image is more subtle
            cv::Mat kernel1 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3,3), cv::Point(1,1));
            cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(1,1));

            if (dilations >= 1)
                cv::dilate(thresh_img, thresh_img, kernel1);
            if (dilations >= 2)
                cv::dilate(thresh_img, thresh_img, kernel2);
            if (dilations >= 3)
                cv::dilate(thresh_img, thresh_img, kernel1);
            if (dilations >= 4)
                cv::dilate(thresh_img, thresh_img, kernel2);
            if (dilations >= 5)
                cv::dilate(thresh_img, thresh_img, kernel1);
            if (dilations >= 6)
                cv::dilate(thresh_img, thresh_img, kernel2);

            // In order to find rectangles that go to the edge, we draw a white
            // line around the image edge. Otherwise FindContours will miss those
            // clipped rectangle contours. The border color will be the image mean,
            // because otherwise we risk screwing up filters like cvSmooth()
            cv::rectangle(thresh_img, cv::Point(0,0),
                          cv::Point(thresh_img.cols - 1, thresh_img.rows - 1),
                          CV_RGB(255,255,255), 3, 8);

            // Generate quadrangles in the following function
            std::vector<ChessboardQuadPtr> quads;

            generateQuads(quads, thresh_img, flags, dilations, true);
            if (quads.empty())
            {
                continue;
            }

            // The following function finds and assigns neighbor quads to every
            // quadrangle in the immediate vicinity fulfilling certain
            // prerequisites
            findQuadNeighbors(quads, dilations);

            // The connected quads will be organized in groups. The following loop
            // increases a "group_idx" identifier.
            // The function "findConnectedQuads assigns all connected quads
            // a unique group ID.
            // If more quadrangles were assigned to a given group (i.e. connected)
            // than are expected by the input variable "patternSize", the
            // function "cleanFoundConnectedQuads" erases the surplus
            // quadrangles by minimizing the convex hull of the remaining pattern.

            for (int group_idx = 0; ; ++group_idx)
            {
                std::vector<ChessboardQuadPtr> quadGroup;

                findConnectedQuads(quads, quadGroup, group_idx, dilations);

                if (quadGroup.empty())
                {
                    break;
                }

                cleanFoundConnectedQuads(quadGroup, patternSize);

                // The following function labels all corners of every quad
                // with a row and column entry.
                // "count" specifies the number of found quads in "quad_group"
                // with group identifier "group_idx"
                // The last parameter is set to "true", because this is the
                // first function call and some initializations need to be
                // made.
                labelQuadGroup(quadGroup, patternSize, true);

                found = checkQuadGroup(quadGroup, outputCorners, patternSize);

                float sumDist = 0;
                int total = 0;

                for (int i = 0; i < (int)outputCorners.size(); ++i)
                {
                    int ni = 0;
                    float avgi = outputCorners.at(i)->meanDist(ni);
                    sumDist += avgi * ni;
                    total += ni;
                }
                prevSqrSize = lround(sumDist / std::max(total, 1));

                if (found && !checkBoardMonotony(outputCorners, patternSize))
                {
                    found = false;
                }
            }
        }
    }

    if (!found)
    {
        return false;
    }
    else
    {
        corners.clear();
        corners.reserve(outputCorners.size());
        for (size_t i = 0; i < outputCorners.size(); ++i)
        {
            corners.push_back(outputCorners.at(i)->pt);
        }

        cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        return true;
    }
}

//===========================================================================
// ERASE OVERHEAD
//===========================================================================
// If we found too many connected quads, remove those which probably do not
// belong.
void
Chessboard::cleanFoundConnectedQuads(std::vector<ChessboardQuadPtr>& quadGroup,
                                     cv::Size patternSize)
{
    cv::Point2f center(0.0f, 0.0f);

    // Number of quads this pattern should contain
    int count = ((patternSize.width + 1)*(patternSize.height + 1) + 1)/2;

    // If we have more quadrangles than we should, try to eliminate duplicates
    // or ones which don't belong to the pattern rectangle. Else go to the end
    // of the function
    if ((int)quadGroup.size() <= count)
    {
        return;
    }

    // Create an array of quadrangle centers
    std::vector<cv::Point2f> centers;
    centers.resize(quadGroup.size());

    for (size_t i = 0; i < quadGroup.size(); ++i)
    {
        cv::Point2f ci(0.0f, 0.0f);
        ChessboardQuadPtr& q = quadGroup[i];

        for (int j = 0; j < 4; ++j)
        {
            ci += q->corners[j]->pt;
        }

        ci *= 0.25f;

        // Centers(i), is the geometric center of quad(i)
        // Center, is the center of all found quads
        centers[i] = ci;

        center += ci;
    }

    center *= 1.0f / quadGroup.size();

    // If we have more quadrangles than we should, we try to eliminate bad
    // ones based on minimizing the bounding box. We iteratively remove the
    // point which reduces the size of the bounding box of the blobs the most
    // (since we want the rectangle to be as small as possible) remove the
    // quadrange that causes the biggest reduction in pattern size until we
    // have the correct number
    while ((int)quadGroup.size() > count)
    {
        double minBoxArea = DBL_MAX;
        int minBoxAreaIndex = -1;

        // For each point, calculate box area without that point
        for (size_t skip = 0; skip < quadGroup.size(); ++skip)
        {
            // get bounding rectangle
            cv::Point2f temp = centers[skip];
            centers[skip] = center;

            std::vector<cv::Point2f> hull;
            cv::convexHull(centers, hull, true, true);
            centers[skip] = temp;

            double hull_area = fabs(cv::contourArea(hull));

            // remember smallest box area
            if (hull_area < minBoxArea)
            {
                minBoxArea = hull_area;
                minBoxAreaIndex = skip;
            }
        }

        ChessboardQuadPtr& q0 = quadGroup[minBoxAreaIndex];

        // remove any references to this quad as a neighbor
        for (size_t i = 0; i < quadGroup.size(); ++i)
        {
            ChessboardQuadPtr& q = quadGroup.at(i);
            for (int j = 0; j < 4; ++j)
            {
                if (q->neighbors[j] == q0)
                {
                    q->neighbors[j].reset();
                    q->count--;
                    for (int k = 0; k < 4; ++k)
                    {
                        if (q0->neighbors[k] == q)
                        {
                            q0->neighbors[k].reset();
                            q0->count--;
                            break;
                        }
                    }
                    break;
                }
            }
        }
        // remove the quad
        quadGroup.at(minBoxAreaIndex) = quadGroup.back();
        centers.at(minBoxAreaIndex) = centers.back();
        quadGroup.pop_back();
        centers.pop_back();
    }
}

//===========================================================================
// FIND COONECTED QUADS
//===========================================================================
void
Chessboard::findConnectedQuads(std::vector<ChessboardQuadPtr>& quads,
                               std::vector<ChessboardQuadPtr>& group,
                               int group_idx, int dilation)
{
    ChessboardQuadPtr q;

    // Scan the array for a first unlabeled quad
    for (size_t i = 0; i < quads.size(); ++i)
    {
        ChessboardQuadPtr& quad = quads.at(i);

        if (quad->count > 0 && quad->group_idx < 0)
        {
            q = quad;
            break;
        }
    }

    if (q.get() == 0)
    {
        return;
    }

    // Recursively find a group of connected quads starting from the seed quad

    std::vector<ChessboardQuadPtr> stack;
    stack.push_back(q);

    group.push_back(q);
    q->group_idx = group_idx;

    while (!stack.empty())
    {
        q = stack.back();
        stack.pop_back();

        for (int i = 0; i < 4; ++i)
        {
            ChessboardQuadPtr& neighbor = q->neighbors[i];

            // If he neighbor exists and the neighbor has more than 0
            // neighbors and the neighbor has not been classified yet.
            if (neighbor.get() && neighbor->count > 0 && neighbor->group_idx < 0)
            {
                stack.push_back(neighbor);
                group.push_back(neighbor);
                neighbor->group_idx = group_idx;
            }
        }
    }
}

void
Chessboard::labelQuadGroup(std::vector<ChessboardQuadPtr>& quadGroup,
                           cv::Size patternSize, bool firstRun)
{
    // If this is the first function call, a seed quad needs to be selected
    if (firstRun)
    {
        // Search for the (first) quad with the maximum number of neighbors
        // (usually 4). This will be our starting point.
        int mark = -1;
        int maxNeighborCount = 0;
        for (size_t i = 0; i < quadGroup.size(); ++i)
        {
            ChessboardQuadPtr& q = quadGroup.at(i);
            if (q->count > maxNeighborCount)
            {
                mark = i;
                maxNeighborCount = q->count;

                if (maxNeighborCount == 4)
                {
                    break;
                }
            }
        }

        // Mark the starting quad's (per definition) upper left corner with
        //(0,0) and then proceed clockwise
        // The following labeling sequence enshures a "right coordinate system"
        ChessboardQuadPtr& q = quadGroup.at(mark);

        q->labeled = true;

        q->corners[0]->row = 0;
        q->corners[0]->column = 0;
        q->corners[1]->row = 0;
        q->corners[1]->column = 1;
        q->corners[2]->row = 1;
        q->corners[2]->column = 1;
        q->corners[3]->row = 1;
        q->corners[3]->column = 0;
    }


    // Mark all other corners with their respective row and column
    bool flagChanged = true;
    while (flagChanged)
    {
        // First reset the flag to "false"
        flagChanged = false;

        // Go through all quads top down is faster, since unlabeled quads will
        // be inserted at the end of the list
        for (int i = quadGroup.size() - 1; i >= 0; --i)
        {
            ChessboardQuadPtr& quad = quadGroup.at(i);

            // Check whether quad "i" has been labeled already
             if (!quad->labeled)
            {
                // Check its neighbors, whether some of them have been labeled
                // already
                for (int j = 0; j < 4; j++)
                {
                    // Check whether the neighbor exists (i.e. is not the NULL
                    // pointer)
                    if (quad->neighbors[j])
                    {
                        ChessboardQuadPtr& quadNeighbor = quad->neighbors[j];

                        // Only proceed, if neighbor "j" was labeled
                        if (quadNeighbor->labeled)
                        {
                            // For every quad it could happen to pass here
                            // multiple times. We therefore "break" later.
                            // Check whitch of the neighbors corners is
                            // connected to the current quad
                            int connectedNeighborCornerId = -1;
                            for (int k = 0; k < 4; ++k)
                            {
                                if (quadNeighbor->neighbors[k] == quad)
                                {
                                    connectedNeighborCornerId = k;

                                    // there is only one, therefore
                                    break;
                                }
                            }


                            // For the following calculations we need the row
                            // and column of the connected neighbor corner and
                            // all other corners of the connected quad "j",
                            // clockwise (CW)
                            ChessboardCornerPtr& conCorner        = quadNeighbor->corners[connectedNeighborCornerId];
                            ChessboardCornerPtr& conCornerCW1     = quadNeighbor->corners[(connectedNeighborCornerId+1)%4];
                            ChessboardCornerPtr& conCornerCW2     = quadNeighbor->corners[(connectedNeighborCornerId+2)%4];
                            ChessboardCornerPtr& conCornerCW3     = quadNeighbor->corners[(connectedNeighborCornerId+3)%4];

                            quad->corners[j]->row            =    conCorner->row;
                            quad->corners[j]->column        =    conCorner->column;
                            quad->corners[(j+1)%4]->row        =    conCorner->row - conCornerCW2->row + conCornerCW3->row;
                            quad->corners[(j+1)%4]->column    =    conCorner->column - conCornerCW2->column + conCornerCW3->column;
                            quad->corners[(j+2)%4]->row        =    conCorner->row + conCorner->row - conCornerCW2->row;
                            quad->corners[(j+2)%4]->column    =    conCorner->column + conCorner->column - conCornerCW2->column;
                            quad->corners[(j+3)%4]->row        =    conCorner->row - conCornerCW2->row + conCornerCW1->row;
                            quad->corners[(j+3)%4]->column    =    conCorner->column - conCornerCW2->column + conCornerCW1->column;

                            // Mark this quad as labeled
                            quad->labeled = true;

                            // Changes have taken place, set the flag
                            flagChanged = true;

                            // once is enough!
                            break;
                        }
                    }
                }
            }
        }
    }


    // All corners are marked with row and column
    // Record the minimal and maximal row and column indices
    // It is unlikely that more than 8bit checkers are used per dimension, if there are
    // an error would have been thrown at the beginning of "cvFindChessboardCorners2"
    int min_row        =  127;
    int max_row        = -127;
    int min_column    =  127;
    int max_column    = -127;

    for (int i = 0; i < (int)quadGroup.size(); ++i)
    {
        ChessboardQuadPtr& q = quadGroup.at(i);

        for (int j = 0; j < 4; ++j)
        {
            ChessboardCornerPtr& c = q->corners[j];

            if (c->row > max_row)
            {
                max_row = c->row;
            }
            if (c->row < min_row)
            {
                min_row = c->row;
            }
            if (c->column > max_column)
            {
                max_column = c->column;
            }
            if (c->column < min_column)
            {
                min_column = c->column;
            }
        }
    }

    // Label all internal corners with "needsNeighbor" = false
    // Label all external corners with "needsNeighbor" = true,
    // except if in a given dimension the pattern size is reached
    for (int i = min_row; i <= max_row; ++i)
    {
        for (int j = min_column; j <= max_column; ++j)
        {
            // A flag that indicates, whether a row/column combination is
            // executed multiple times
            bool flag = false;

            // Remember corner and quad
            int cornerID;
            int quadID;

            for (int k = 0; k < (int)quadGroup.size(); ++k)
            {
                ChessboardQuadPtr& q = quadGroup.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    if ((q->corners[l]->row == i) && (q->corners[l]->column == j))
                    {
                        if (flag)
                        {
                            // Passed at least twice through here
                            q->corners[l]->needsNeighbor = false;
                            quadGroup[quadID]->corners[cornerID]->needsNeighbor = false;
                        }
                        else
                        {
                            // Mark with needs a neighbor, but note the
                            // address
                            q->corners[l]->needsNeighbor = true;
                            cornerID = l;
                            quadID = k;
                        }

                        // set the flag to true
                        flag = true;
                    }
                }
            }
        }
    }

    // Complete Linking:
    // sometimes not all corners were properly linked in "findQuadNeighbors",
    // but after labeling each corner with its respective row and column, it is
    // possible to match them anyway.
    for (int i = min_row; i <= max_row; ++i)
    {
        for (int j = min_column; j <= max_column; ++j)
        {
            // the following "number" indicates the number of corners which
            // correspond to the given (i,j) value
            // 1    is a border corner or a conrer which still needs a neighbor
            // 2    is a fully connected internal corner
            // >2    something went wrong during labeling, report a warning
            int number = 1;

            // remember corner and quad
            int cornerID;
            int quadID;

            for (int k = 0; k < (int)quadGroup.size(); ++k)
            {
                ChessboardQuadPtr& q = quadGroup.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    if ((q->corners[l]->row == i) && (q->corners[l]->column == j))
                    {

                        if (number == 1)
                        {
                            // First corner, note its ID
                            cornerID = l;
                            quadID = k;
                        }

                        else if (number == 2)
                        {
                            // Second corner, check wheter this and the
                            // first one have equal coordinates, else
                            // interpolate
                            cv::Point2f delta = q->corners[l]->pt - quadGroup[quadID]->corners[cornerID]->pt;

                            if (delta.x != 0.0f || delta.y != 0.0f)
                            {
                                // Interpolate
                                q->corners[l]->pt -= delta * 0.5f;

                                quadGroup[quadID]->corners[cornerID]->pt += delta * 0.5f;
                            }
                        }
                        else if (number > 2)
                        {
                            // Something went wrong during row/column labeling
                            // Report a Warning
                            // ->Implemented in the function "mrWriteCorners"
                        }

                        // increase the number by one
                        ++number;
                    }
                }
            }
        }
    }


    // Bordercorners don't need any neighbors, if the pattern size in the
    // respective direction is reached
    // The only time we can make shure that the target pattern size is reached in a given
    // dimension, is when the larger side has reached the target size in the maximal
    // direction, or if the larger side is larger than the smaller target size and the
    // smaller side equals the smaller target size
    int largerDimPattern = std::max(patternSize.height,patternSize.width);
    int smallerDimPattern = std::min(patternSize.height,patternSize.width);
    bool flagSmallerDim1 = false;
    bool flagSmallerDim2 = false;

    if ((largerDimPattern + 1) == max_column - min_column)
    {
        flagSmallerDim1 = true;
        // We found out that in the column direction the target pattern size is reached
        // Therefore border column corners do not need a neighbor anymore
        // Go through all corners
        for (int k = 0; k < (int)quadGroup.size(); ++k)
        {
            ChessboardQuadPtr& q = quadGroup.at(k);

            for (int l = 0; l < 4; ++l)
            {
                ChessboardCornerPtr& c = q->corners[l];

                if (c->column == min_column || c->column == max_column)
                {
                    // Needs no neighbor anymore
                    c->needsNeighbor = false;
                }
            }
        }
    }

    if ((largerDimPattern + 1) == max_row - min_row)
    {
        flagSmallerDim2 = true;
        // We found out that in the column direction the target pattern size is reached
        // Therefore border column corners do not need a neighbor anymore
        // Go through all corners
        for (int k = 0; k < (int)quadGroup.size(); ++k)
        {
            ChessboardQuadPtr& q = quadGroup.at(k);

            for (int l = 0; l < 4; ++l)
            {
                ChessboardCornerPtr& c = q->corners[l];

                if (c->row == min_row || c->row == max_row)
                {
                    // Needs no neighbor anymore
                    c->needsNeighbor = false;
                }
            }
        }
    }


    // Check the two flags:
    //    -    If one is true and the other false, then the pattern target
    //        size was reached in in one direction -> We can check, whether the target
    //        pattern size is also reached in the other direction
    //  -    If both are set to true, then we deal with a square board -> do nothing
    //  -    If both are set to false -> There is a possibility that the larger side is
    //        larger than the smaller target size -> Check and if true, then check whether
    //        the other side has the same size as the smaller target size
    if ((flagSmallerDim1 == false && flagSmallerDim2 == true))
    {
        // Larger target pattern size is in row direction, check wheter smaller target
        // pattern size is reached in column direction
        if ((smallerDimPattern + 1) == max_column - min_column)
        {
            for (int k = 0; k < (int)quadGroup.size(); ++k)
            {
                ChessboardQuadPtr& q = quadGroup.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    ChessboardCornerPtr& c = q->corners[l];

                    if (c->column == min_column || c->column == max_column)
                    {
                        // Needs no neighbor anymore
                        c->needsNeighbor = false;
                    }
                }
            }
        }
    }

    if ((flagSmallerDim1 == true && flagSmallerDim2 == false))
    {
        // Larger target pattern size is in column direction, check wheter smaller target
        // pattern size is reached in row direction
        if ((smallerDimPattern + 1) == max_row - min_row)
        {
            for (int k = 0; k < (int)quadGroup.size(); ++k)
            {
                ChessboardQuadPtr& q = quadGroup.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    ChessboardCornerPtr& c = q->corners[l];

                    if (c->row == min_row || c->row == max_row)
                    {
                        // Needs no neighbor anymore
                        c->needsNeighbor = false;
                    }
                }
            }
        }
    }

    if ((flagSmallerDim1 == false && flagSmallerDim2 == false) && smallerDimPattern + 1 < max_column - min_column)
    {
        // Larger target pattern size is in column direction, check wheter smaller target
        // pattern size is reached in row direction
        if ((smallerDimPattern + 1) == max_row - min_row)
        {
            for (int k = 0; k < (int)quadGroup.size(); ++k)
            {
                ChessboardQuadPtr& q = quadGroup.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    ChessboardCornerPtr& c = q->corners[l];

                    if (c->row == min_row || c->row == max_row)
                    {
                        // Needs no neighbor anymore
                        c->needsNeighbor = false;
                    }
                }
            }
        }
    }

    if ((flagSmallerDim1 == false && flagSmallerDim2 == false) && smallerDimPattern + 1 < max_row - min_row)
    {
        // Larger target pattern size is in row direction, check wheter smaller target
        // pattern size is reached in column direction
        if ((smallerDimPattern + 1) == max_column - min_column)
        {
            for (int k = 0; k < (int)quadGroup.size(); ++k)
            {
                ChessboardQuadPtr& q = quadGroup.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    ChessboardCornerPtr& c = q->corners[l];

                    if (c->column == min_column || c->column == max_column)
                    {
                        // Needs no neighbor anymore
                        c->needsNeighbor = false;
                    }
                }
            }
        }
    }
}

//===========================================================================
// GIVE A GROUP IDX
//===========================================================================
void
Chessboard::findQuadNeighbors(std::vector<ChessboardQuadPtr>& quads, int dilation)
{
    // Thresh dilation is used to counter the effect of dilation on the
    // distance between 2 neighboring corners. Since the distance below is
    // computed as its square, we do here the same. Additionally, we take the
    // conservative assumption that dilation was performed using the 3x3 CROSS
    // kernel, which coresponds to the 4-neighborhood.
    const float thresh_dilation = (float)(2*dilation+3)*(2*dilation+3)*2;    // the "*2" is for the x and y component
                                                                            // the "3" is for initial corner mismatch

    // Find quad neighbors
    for (size_t idx = 0; idx < quads.size(); ++idx)
    {
        ChessboardQuadPtr& curQuad = quads.at(idx);

        // Go through all quadrangles and label them in groups
        // For each corner of this quadrangle
        for (int i = 0; i < 4; ++i)
        {
            float minDist = FLT_MAX;
            int closestCornerIdx = -1;
            ChessboardQuadPtr closestQuad;

            if (curQuad->neighbors[i])
            {
                continue;
            }

            cv::Point2f pt = curQuad->corners[i]->pt;

            // Find the closest corner in all other quadrangles
            for (size_t k = 0; k < quads.size(); ++k)
            {
                if (k == idx)
                {
                    continue;
                }

                ChessboardQuadPtr& quad = quads.at(k);

                for (int j = 0; j < 4; ++j)
                {
                    // If it already has a neighbor
                    if (quad->neighbors[j])
                    {
                        continue;
                    }

                    cv::Point2f dp = pt - quad->corners[j]->pt;
                    float dist = dp.dot(dp);

                    // The following "if" checks, whether "dist" is the
                    // shortest so far and smaller than the smallest
                    // edge length of the current and target quads
                    if (dist < minDist &&
                        dist <= (curQuad->edge_len + thresh_dilation) &&
                        dist <= (quad->edge_len + thresh_dilation)   )
                    {
                        // Check whether conditions are fulfilled
                        if (matchCorners(curQuad, i, quad, j))
                        {
                            closestCornerIdx = j;
                            closestQuad = quad;
                            minDist = dist;
                        }
                    }
                }
            }

            // Have we found a matching corner point?
            if (closestCornerIdx >= 0 && minDist < FLT_MAX)
            {
                ChessboardCornerPtr closestCorner = closestQuad->corners[closestCornerIdx];

                // Make sure that the closest quad does not have the current
                // quad as neighbor already
                bool valid = true;
                for (int j = 0; j < 4; ++j)
                {
                    if (closestQuad->neighbors[j] == curQuad)
                    {
                        valid = false;
                        break;
                    }
                }
                if (!valid)
                {
                    continue;
                }

                // We've found one more corner - remember it
                closestCorner->pt = (pt + closestCorner->pt) * 0.5f;

                curQuad->count++;
                curQuad->neighbors[i] = closestQuad;
                curQuad->corners[i] = closestCorner;

                closestQuad->count++;
                closestQuad->neighbors[closestCornerIdx] = curQuad;
                closestQuad->corners[closestCornerIdx] = closestCorner;
            }
        }
    }
}



//===========================================================================
// AUGMENT PATTERN WITH ADDITIONAL QUADS
//===========================================================================
// The first part of the function is basically a copy of
// "findQuadNeighbors"
// The comparisons between two points and two lines could be computed in their
// own function
int
Chessboard::augmentBestRun(std::vector<ChessboardQuadPtr>& candidateQuads, int candidateDilation,
                           std::vector<ChessboardQuadPtr>& existingQuads, int existingDilation)
{
    // thresh dilation is used to counter the effect of dilation on the
    // distance between 2 neighboring corners. Since the distance below is
    // computed as its square, we do here the same. Additionally, we take the
    // conservative assumption that dilation was performed using the 3x3 CROSS
    // kernel, which coresponds to the 4-neighborhood.
    const float thresh_dilation = (2*candidateDilation+3)*(2*existingDilation+3)*2;    // the "*2" is for the x and y component

    // Search all old quads which have a neighbor that needs to be linked
    for (size_t idx = 0; idx < existingQuads.size(); ++idx)
    {
        ChessboardQuadPtr& curQuad = existingQuads.at(idx);

        // For each corner of this quadrangle
        for (int i = 0; i < 4; ++i)
        {
            float minDist = FLT_MAX;
            int closestCornerIdx = -1;
            ChessboardQuadPtr closestQuad;

            // If curQuad corner[i] is already linked, continue
            if (!curQuad->corners[i]->needsNeighbor)
            {
                continue;
            }

            cv::Point2f pt = curQuad->corners[i]->pt;

            // Look for a match in all candidateQuads' corners
            for (size_t k = 0; k < candidateQuads.size(); ++k)
            {
                ChessboardQuadPtr& candidateQuad = candidateQuads.at(k);

                // Only look at unlabeled new quads
                if (candidateQuad->labeled)
                {
                    continue;
                }

                for (int j = 0; j < 4; ++j)
                {
                    // Only proceed if they are less than dist away from each
                    // other
                    cv::Point2f dp = pt - candidateQuad->corners[j]->pt;
                    float dist = dp.dot(dp);

                    if ((dist < minDist) &&
                        dist <= (curQuad->edge_len + thresh_dilation) &&
                        dist <= (candidateQuad->edge_len + thresh_dilation))
                    {
                        if (matchCorners(curQuad, i, candidateQuad, j))
                        {
                            closestCornerIdx = j;
                            closestQuad = candidateQuad;
                            minDist = dist;
                        }
                    }
                }
            }

            // Have we found a matching corner point?
            if (closestCornerIdx >= 0 && minDist < FLT_MAX)
            {
                ChessboardCornerPtr closestCorner = closestQuad->corners[closestCornerIdx];
                closestCorner->pt = (pt + closestCorner->pt) * 0.5f;

                // We've found one more corner - remember it
                // ATTENTION: write the corner x and y coordinates separately,
                // else the crucial row/column entries will be overwritten !!!
                curQuad->corners[i]->pt = closestCorner->pt;
                curQuad->neighbors[i] = closestQuad;
                closestQuad->corners[closestCornerIdx]->pt = closestCorner->pt;

                // Label closest quad as labeled. In this way we exclude it
                // being considered again during the next loop iteration
                closestQuad->labeled = true;

                // We have a new member of the final pattern, copy it over
                ChessboardQuadPtr newQuad(new ChessboardQuad);
                newQuad->count        = 1;
                newQuad->edge_len    = closestQuad->edge_len;
                newQuad->group_idx    = curQuad->group_idx;    //the same as the current quad
                newQuad->labeled    = false;                //do it right afterwards

                curQuad->neighbors[i] = newQuad;

                // We only know one neighbor for sure
                newQuad->neighbors[closestCornerIdx] = curQuad;

                for (int j = 0; j < 4; j++)
                {
                    newQuad->corners[j].reset(new ChessboardCorner);
                    newQuad->corners[j]->pt = closestQuad->corners[j]->pt;
                }

                existingQuads.push_back(newQuad);

                // Start the function again
                return -1;
            }
        }
    }

    // Finished, don't start the function again
    return 1;
}



//===========================================================================
// GENERATE QUADRANGLES
//===========================================================================
void
Chessboard::generateQuads(std::vector<ChessboardQuadPtr>& quads,
                          cv::Mat& image, int flags,
                          int dilation, bool firstRun)
{
    // Empirical lower bound for the size of allowable quadrangles.
    // MARTIN, modified: Added "*0.1" in order to find smaller quads.
    int minSize = lround(image.cols * image.rows * .03 * 0.01 * 0.92 * 0.1);

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    // Initialize contour retrieving routine
    cv::findContours(image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    std::vector< std::vector<cv::Point> > quadContours;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        // Reject contours with a too small perimeter and contours which are
        // completely surrounded by another contour
        // MARTIN: If this function is called during PART 1, then the parameter "first run"
        // is set to "true". This guarantees, that only "nice" squares are detected.
        // During PART 2, we allow the polygonal approximation function below to
        // approximate more freely, which can result in recognized "squares" that are in
        // reality multiple blurred and sticked together squares.
        std::vector<cv::Point>& contour = contours.at(i);

        if (hierarchy[i][3] == -1 || cv::contourArea(contour) < minSize)
        {
            continue;
        }

        int min_approx_level = 1, max_approx_level;
        if (firstRun)
        {
            max_approx_level = 3;
        }
        else
        {
            max_approx_level = MAX_CONTOUR_APPROX;
        }

        std::vector<cv::Point> approxContour;
        for (int approx_level = min_approx_level; approx_level <= max_approx_level; approx_level++)
        {
            cv::approxPolyDP(contour, approxContour, approx_level, true);

            if (approxContour.size() == 4)
            {
                break;
            }
        }

        // Reject non-quadrangles
        if (approxContour.size() == 4 && cv::isContourConvex(approxContour))
        {
            double p = cv::arcLength(approxContour, true);
            double area = cv::contourArea(approxContour);

            cv::Point pt[4];
            for (int i = 0; i < 4; i++)
            {
                pt[i] = approxContour[i];
            }

            cv::Point dp = pt[0] - pt[2];
            double d1 = sqrt(dp.dot(dp));

            dp = pt[1] - pt[3];
            double d2 = sqrt(dp.dot(dp));

            // PHILIPG: Only accept those quadrangles which are more
            // square than rectangular and which are big enough
            dp = pt[0] - pt[1];
            double d3 = sqrt(dp.dot(dp));
            dp = pt[1] - pt[2];
            double d4 = sqrt(dp.dot(dp));

            if (!(flags & cv::CALIB_CB_FILTER_QUADS) ||
                (d3*4 > d4 && d4*4 > d3 && d3*d4 < area*1.5 && area > minSize &&
                d1 >= 0.15 * p && d2 >= 0.15 * p))
            {
                quadContours.push_back(approxContour);
            }
        }
    }

    // Allocate quad & corner buffers
    quads.resize(quadContours.size());

    // Create array of quads structures
    for (size_t idx = 0; idx < quadContours.size(); ++idx)
    {
        ChessboardQuadPtr& q = quads.at(idx);
        std::vector<cv::Point>& contour = quadContours.at(idx);

        // Reset group ID
        q.reset(new ChessboardQuad);
        assert(contour.size() == 4);

        for (int i = 0; i < 4; ++i)
        {
            cv::Point2f pt = contour.at(i);
            ChessboardCornerPtr corner(new ChessboardCorner);
            corner->pt = pt;
            q->corners[i] = corner;
        }

        for (int i = 0; i < 4; ++i)
        {
            cv::Point2f dp = q->corners[i]->pt - q->corners[(i+1)&3]->pt;
            float d = dp.dot(dp);
            if (q->edge_len > d)
            {
                q->edge_len = d;
            }
        }
    }
}

bool
Chessboard::checkQuadGroup(std::vector<ChessboardQuadPtr>& quads,
                           std::vector<ChessboardCornerPtr>& corners,
                           cv::Size patternSize)
{
    // Initialize
    bool flagRow = false;
    bool flagColumn = false;
    int height = -1;
    int width = -1;

    // Compute the minimum and maximum row / column ID
    // (it is unlikely that more than 8bit checkers are used per dimension)
    int min_row    =  127;
    int max_row    = -127;
    int min_col    =  127;
    int max_col    = -127;

    for (size_t i = 0; i < quads.size(); ++i)
    {
        ChessboardQuadPtr& q = quads.at(i);

        for (int j = 0; j < 4; ++j)
        {
            ChessboardCornerPtr& c = q->corners[j];

            if (c->row > max_row)
            {
                max_row = c->row;
            }
            if (c->row < min_row)
            {
                min_row = c->row;
            }
            if (c->column > max_col)
            {
                max_col = c->column;
            }
            if (c->column < min_col)
            {
                min_col = c->column;
            }
        }
    }

    // If in a given direction the target pattern size is reached, we know exactly how
    // the checkerboard is oriented.
    // Else we need to prepare enough "dummy" corners for the worst case.
    for (size_t i = 0; i < quads.size(); ++i)
    {
        ChessboardQuadPtr& q = quads.at(i);

        for (int j = 0; j < 4; ++j)
        {
            ChessboardCornerPtr& c = q->corners[j];

            if (c->column == max_col && c->row != min_row && c->row != max_row && !c->needsNeighbor)
            {
                flagColumn = true;
            }
            if (c->row == max_row && c->column != min_col && c->column != max_col && !c->needsNeighbor)
            {
                flagRow = true;
            }
        }
    }

    if (flagColumn)
    {
        if (max_col - min_col == patternSize.width + 1)
        {
            width = patternSize.width;
            height = patternSize.height;
        }
        else
        {
            width = patternSize.height;
            height = patternSize.width;
        }
    }
    else if (flagRow)
    {
        if (max_row - min_row == patternSize.width + 1)
        {
            height = patternSize.width;
            width = patternSize.height;
        }
        else
        {
            height = patternSize.height;
            width = patternSize.width;
        }
    }
    else
    {
        // If target pattern size is not reached in at least one of the two
        // directions,  then we do not know where the remaining corners are
        // located. Account for this.
        width = std::max(patternSize.width, patternSize.height);
        height = std::max(patternSize.width, patternSize.height);
    }

    ++min_row;
    ++min_col;
    max_row = min_row + height - 1;
    max_col = min_col + width - 1;

    corners.clear();

    int linkedBorderCorners = 0;

    // Write the corners in increasing order to the output file
    for (int i = min_row; i <= max_row; ++i)
    {
        for (int j = min_col; j <= max_col; ++j)
        {
            // Reset the iterator
            int iter = 1;

            for (int k = 0; k < (int)quads.size(); ++k)
            {
                ChessboardQuadPtr& quad = quads.at(k);

                for (int l = 0; l < 4; ++l)
                {
                    ChessboardCornerPtr& c = quad->corners[l];

                    if (c->row == i && c->column == j)
                    {
                        bool boardEdge = false;
                        if (i == min_row || i == max_row ||
                            j == min_col || j == max_col)
                        {
                            boardEdge = true;
                        }

                        if ((iter == 1 && boardEdge) || (iter == 2 && !boardEdge))
                        {
                            // The respective row and column have been found
                            corners.push_back(quad->corners[l]);
                        }

                        if (iter == 2 && boardEdge)
                        {
                            ++linkedBorderCorners;
                        }

                        // If the iterator is larger than two, this means that more than
                        // two corners have the same row / column entries. Then some
                        // linking errors must have occured and we should not use the found
                        // pattern
                        if (iter > 2)
                        {
                            return false;
                        }

                        iter++;
                    }
                }
            }
        }
    }

    if ((int)corners.size() != patternSize.width * patternSize.height ||
        linkedBorderCorners < (patternSize.width * 2 + patternSize.height * 2 - 2) * 0.75f)
    {
        return false;
    }

    // check that no corners lie at image boundary
    float border = 5.0f;
    for (int i = 0; i < (int)corners.size(); ++i)
    {
        ChessboardCornerPtr& c = corners.at(i);

        if (c->pt.x < border || c->pt.x > mImage.cols - border ||
            c->pt.y < border || c->pt.y > mImage.rows - border)
        {
            return false;
        }
    }

    // check if we need to transpose the board
    if (width != patternSize.width)
    {
        std::swap(width, height);

        std::vector<ChessboardCornerPtr> outputCorners;
        outputCorners.resize(corners.size());

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                outputCorners.at(i * width + j) = corners.at(j * height + i);
            }
        }

        corners = outputCorners;
    }

    // check if we need to revert the order in each row
    cv::Point2f p0 = corners.at(0)->pt;
    cv::Point2f p1 = corners.at(width-1)->pt;
    cv::Point2f p2 = corners.at(width)->pt;

    if ((p1 - p0).cross(p2 - p0) < 0.0f)
    {
        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width / 2; ++j)
            {
                std::swap(corners.at(i * width + j), corners.at(i * width + width - j - 1));
            }
        }
    }

    p0 = corners.at(0)->pt;
    p2 = corners.at(width)->pt;

    // check if we need to rotate the board
    if (p2.y < p0.y)
    {
        std::vector<ChessboardCornerPtr> outputCorners;
        outputCorners.resize(corners.size());

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                outputCorners.at(i * width + j) = corners.at((height - i - 1) * width + width - j - 1);
            }
        }

        corners = outputCorners;
    }

    return true;
}

void
Chessboard::getQuadrangleHypotheses(const std::vector< std::vector<cv::Point> >& contours,
                                    std::vector< std::pair<float, int> >& quads,
                                    int classId) const
{
    const float minAspectRatio = 0.2f;
    const float maxAspectRatio = 5.0f;
    const float minBoxSize = 10.0f;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        cv::RotatedRect box = cv::minAreaRect(contours.at(i));
        float boxSize = std::max(box.size.width, box.size.height);
        if (boxSize < minBoxSize)
        {
            continue;
        }

        float aspectRatio = box.size.width / std::max(box.size.height, 1.0f);

        if (aspectRatio < minAspectRatio || aspectRatio > maxAspectRatio)
        {
            continue;
        }

        quads.push_back(std::pair<float, int>(boxSize, classId));
    }
}

bool less_pred(const std::pair<float, int>& p1, const std::pair<float, int>& p2)
{
    return p1.first < p2.first;
}

void countClasses(const std::vector<std::pair<float, int> >& pairs, size_t idx1, size_t idx2, std::vector<int>& counts)
{
    counts.assign(2, 0);
    for (size_t i = idx1; i != idx2; ++i)
    {
        counts[pairs[i].second]++;
    }
}

bool
Chessboard::checkChessboard(const cv::Mat& image, cv::Size patternSize) const
{
    const int erosionCount = 1;
    const float blackLevel = 20.f;
    const float whiteLevel = 130.f;
    const float blackWhiteGap = 70.f;

    cv::Mat white = image.clone();

    cv::Mat black = image.clone();

    cv::erode(white, white, cv::Mat(), cv::Point(-1,-1), erosionCount);
    cv::dilate(black, black, cv::Mat(), cv::Point(-1,-1), erosionCount);

    cv::Mat thresh(image.rows, image.cols, CV_8UC1);

    bool result = false;
    for (float threshLevel = blackLevel; threshLevel < whiteLevel && !result; threshLevel += 20.0f)
    {
        cv::threshold(white, thresh, threshLevel + blackWhiteGap, 255, cv::THRESH_BINARY);

        std::vector< std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        // Initialize contour retrieving routine
        cv::findContours(thresh, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

        std::vector<std::pair<float, int> > quads;
        getQuadrangleHypotheses(contours, quads, 1);

        cv::threshold(black, thresh, threshLevel, 255, cv::THRESH_BINARY_INV);

        cv::findContours(thresh, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
        getQuadrangleHypotheses(contours, quads, 0);

        const size_t min_quads_count = patternSize.width * patternSize.height / 2;
        std::sort(quads.begin(), quads.end(), less_pred);

        // now check if there are many hypotheses with similar sizes
        // do this by floodfill-style algorithm
        const float sizeRelDev = 0.4f;

        for (size_t i = 0; i < quads.size(); ++i)
        {
            size_t j = i + 1;
            for (; j < quads.size(); ++j)
            {
                if (quads[j].first / quads[i].first > 1.0f + sizeRelDev)
                {
                    break;
                }
            }

            if (j + 1 > min_quads_count + i)
            {
                // check the number of black and white squares
                std::vector<int> counts;
                countClasses(quads, i, j, counts);
                const int blackCount = lroundf(ceilf(patternSize.width / 2.0f) * ceilf(patternSize.height / 2.0f));
                const int whiteCount = lroundf(floorf(patternSize.width / 2.0f) * floorf(patternSize.height / 2.0f));
                if (counts[0] < blackCount * 0.75f ||
                    counts[1] < whiteCount * 0.75f)
                {
                    continue;
                }
                result = true;
                break;
            }
        }
    }

    return result;
}

bool
Chessboard::checkBoardMonotony(std::vector<ChessboardCornerPtr>& corners,
                               cv::Size patternSize)
{
    const float threshFactor = 0.2f;

    Spline splineXY, splineYX;
    splineXY.setLowBC(Spline::PARABOLIC_RUNOUT_BC);
    splineXY.setHighBC(Spline::PARABOLIC_RUNOUT_BC);
    splineYX.setLowBC(Spline::PARABOLIC_RUNOUT_BC);
    splineYX.setHighBC(Spline::PARABOLIC_RUNOUT_BC);

    // check if each row of corners approximates a cubic spline
    for (int i = 0; i < patternSize.height; ++i)
    {
        splineXY.clear();
        splineYX.clear();

        cv::Point2f p[3];
        p[0] = corners.at(i * patternSize.width)->pt;
        p[1] = corners.at(i * patternSize.width + patternSize.width / 2)->pt;
        p[2] = corners.at(i * patternSize.width + patternSize.width - 1)->pt;

        for (int j = 0; j < 3; ++j)
        {
            splineXY.addPoint(p[j].x, p[j].y);
            splineYX.addPoint(p[j].y, p[j].x);
        }

        for (int j = 1; j < patternSize.width - 1; ++j)
        {
            cv::Point2f& p_j = corners.at(i * patternSize.width + j)->pt;

            float thresh = std::numeric_limits<float>::max();

            // up-neighbor
            if (i > 0)
            {
                cv::Point2f& neighbor = corners.at((i - 1) * patternSize.width + j)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_j));
            }
            // down-neighbor
            if (i < patternSize.height - 1)
            {
                cv::Point2f& neighbor = corners.at((i + 1) * patternSize.width + j)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_j));
            }
            // left-neighbor
            {
                cv::Point2f& neighbor = corners.at(i * patternSize.width + j - 1)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_j));
            }
            // right-neighbor
            {
                cv::Point2f& neighbor = corners.at(i * patternSize.width + j + 1)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_j));
            }

            thresh *= threshFactor;

            if (fminf(fabsf(splineXY(p_j.x) - p_j.y), fabsf(splineYX(p_j.y) - p_j.x)) > thresh)
            {
                return false;
            }
        }
    }

    // check if each column of corners approximates a cubic spline
    for (int j = 0; j < patternSize.width; ++j)
    {
        splineXY.clear();
        splineYX.clear();

        cv::Point2f p[3];
        p[0] = corners.at(j)->pt;
        p[1] = corners.at(patternSize.height / 2 * patternSize.width + j)->pt;
        p[2] = corners.at((patternSize.height - 1) * patternSize.width + j)->pt;

        for (int i = 0; i < 3; ++i)
        {
            splineXY.addPoint(p[i].x, p[i].y);
            splineYX.addPoint(p[i].y, p[i].x);
        }

        for (int i = 1; i < patternSize.height - 1; ++i)
        {
            cv::Point2f& p_i = corners.at(i * patternSize.width + j)->pt;

            float thresh = std::numeric_limits<float>::max();

            // up-neighbor
            {
                cv::Point2f& neighbor = corners.at((i - 1) * patternSize.width + j)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_i));
            }
            // down-neighbor
            {
                cv::Point2f& neighbor = corners.at((i + 1) * patternSize.width + j)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_i));
            }
            // left-neighbor
            if (j > 0)
            {
                cv::Point2f& neighbor = corners.at(i * patternSize.width + j - 1)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_i));
            }
            // right-neighbor
            if (j < patternSize.width - 1)
            {
                cv::Point2f& neighbor = corners.at(i * patternSize.width + j + 1)->pt;
                thresh = fminf(thresh, cv::norm(neighbor - p_i));
            }

            thresh *= threshFactor;

            if (fminf(fabsf(splineXY(p_i.x) - p_i.y), fabsf(splineYX(p_i.y) - p_i.x)) > thresh)
            {
                return false;
            }
        }
    }

    return true;
}

bool
Chessboard::matchCorners(ChessboardQuadPtr& quad1, int corner1,
                         ChessboardQuadPtr& quad2, int corner2) const
{
    // First Check everything from the viewpoint of the
    // current quad compute midpoints of "parallel" quad
    // sides 1
    float x1 = (quad1->corners[corner1]->pt.x + quad1->corners[(corner1+1)%4]->pt.x)/2;
    float y1 = (quad1->corners[corner1]->pt.y + quad1->corners[(corner1+1)%4]->pt.y)/2;
    float x2 = (quad1->corners[(corner1+2)%4]->pt.x + quad1->corners[(corner1+3)%4]->pt.x)/2;
    float y2 = (quad1->corners[(corner1+2)%4]->pt.y + quad1->corners[(corner1+3)%4]->pt.y)/2;
    // compute midpoints of "parallel" quad sides 2
    float x3 = (quad1->corners[corner1]->pt.x + quad1->corners[(corner1+3)%4]->pt.x)/2;
    float y3 = (quad1->corners[corner1]->pt.y + quad1->corners[(corner1+3)%4]->pt.y)/2;
    float x4 = (quad1->corners[(corner1+1)%4]->pt.x + quad1->corners[(corner1+2)%4]->pt.x)/2;
    float y4 = (quad1->corners[(corner1+1)%4]->pt.y + quad1->corners[(corner1+2)%4]->pt.y)/2;

    // MARTIN: Heuristic
    // For corner2 of quad2 to be considered,
    // it needs to be on the same side of the two lines as
    // corner1. This is given, if the cross product has
    // the same sign for both computations below:
    float a1 = x1 - x2;
    float b1 = y1 - y2;
    // the current corner
    float c11 = quad1->corners[corner1]->pt.x - x2;
    float d11 = quad1->corners[corner1]->pt.y - y2;
    // the candidate corner
    float c12 = quad2->corners[corner2]->pt.x - x2;
    float d12 = quad2->corners[corner2]->pt.y - y2;
    float sign11 = a1*d11 - c11*b1;
    float sign12 = a1*d12 - c12*b1;

    float a2 = x3 - x4;
    float b2 = y3 - y4;
    // the current corner
    float c21 = quad1->corners[corner1]->pt.x - x4;
    float d21 = quad1->corners[corner1]->pt.y - y4;
    // the candidate corner
    float c22 = quad2->corners[corner2]->pt.x - x4;
    float d22 = quad2->corners[corner2]->pt.y - y4;
    float sign21 = a2*d21 - c21*b2;
    float sign22 = a2*d22 - c22*b2;

    // Also make shure that two border quads of the same row or
    // column don't link. Check from the current corner's view,
    // whether the corner diagonal from the candidate corner
    // is also on the same side of the two lines as the current
    // corner and the candidate corner.
    float c13 = quad2->corners[(corner2+2)%4]->pt.x - x2;
    float d13 = quad2->corners[(corner2+2)%4]->pt.y - y2;
    float c23 = quad2->corners[(corner2+2)%4]->pt.x - x4;
    float d23 = quad2->corners[(corner2+2)%4]->pt.y - y4;
    float sign13 = a1*d13 - c13*b1;
    float sign23 = a2*d23 - c23*b2;


    // Second: Then check everything from the viewpoint of
    // the candidate quad. Compute midpoints of "parallel"
    // quad sides 1
    float u1 = (quad2->corners[corner2]->pt.x + quad2->corners[(corner2+1)%4]->pt.x)/2;
    float v1 = (quad2->corners[corner2]->pt.y + quad2->corners[(corner2+1)%4]->pt.y)/2;
    float u2 = (quad2->corners[(corner2+2)%4]->pt.x + quad2->corners[(corner2+3)%4]->pt.x)/2;
    float v2 = (quad2->corners[(corner2+2)%4]->pt.y + quad2->corners[(corner2+3)%4]->pt.y)/2;
    // compute midpoints of "parallel" quad sides 2
    float u3 = (quad2->corners[corner2]->pt.x + quad2->corners[(corner2+3)%4]->pt.x)/2;
    float v3 = (quad2->corners[corner2]->pt.y + quad2->corners[(corner2+3)%4]->pt.y)/2;
    float u4 = (quad2->corners[(corner2+1)%4]->pt.x + quad2->corners[(corner2+2)%4]->pt.x)/2;
    float v4 = (quad2->corners[(corner2+1)%4]->pt.y + quad2->corners[(corner2+2)%4]->pt.y)/2;

    // MARTIN: Heuristic
    // For corner2 of quad2 to be considered,
    // it needs to be on the same side of the two lines as
    // corner1. This is given, if the cross product has
    // the same sign for both computations below:
    float a3 = u1 - u2;
    float b3 = v1 - v2;
    // the current corner
    float c31 = quad1->corners[corner1]->pt.x - u2;
    float d31 = quad1->corners[corner1]->pt.y - v2;
    // the candidate corner
    float c32 = quad2->corners[corner2]->pt.x - u2;
    float d32 = quad2->corners[corner2]->pt.y - v2;
    float sign31 = a3*d31-c31*b3;
    float sign32 = a3*d32-c32*b3;

    float a4 = u3 - u4;
    float b4 = v3 - v4;
    // the current corner
    float c41 = quad1->corners[corner1]->pt.x - u4;
    float d41 = quad1->corners[corner1]->pt.y - v4;
    // the candidate corner
    float c42 = quad2->corners[corner2]->pt.x - u4;
    float d42 = quad2->corners[corner2]->pt.y - v4;
    float sign41 = a4*d41-c41*b4;
    float sign42 = a4*d42-c42*b4;

    // Also make sure that two border quads of the same row or
    // column don't link. Check from the candidate corner's view,
    // whether the corner diagonal from the current corner
    // is also on the same side of the two lines as the current
    // corner and the candidate corner.
    float c33 = quad1->corners[(corner1+2)%4]->pt.x - u2;
    float d33 = quad1->corners[(corner1+2)%4]->pt.y - v2;
    float c43 = quad1->corners[(corner1+2)%4]->pt.x - u4;
    float d43 = quad1->corners[(corner1+2)%4]->pt.y - v4;
    float sign33 = a3*d33-c33*b3;
    float sign43 = a4*d43-c43*b4;


    // This time we also need to make shure, that no quad
    // is linked to a quad of another dilation run which
    // may lie INSIDE it!!!
    // Third: Therefore check everything from the viewpoint
    // of the current quad compute midpoints of "parallel"
    // quad sides 1
    float x5 = quad1->corners[corner1]->pt.x;
    float y5 = quad1->corners[corner1]->pt.y;
    float x6 = quad1->corners[(corner1+1)%4]->pt.x;
    float y6 = quad1->corners[(corner1+1)%4]->pt.y;
    // compute midpoints of "parallel" quad sides 2
    float x7 = x5;
    float y7 = y5;
    float x8 = quad1->corners[(corner1+3)%4]->pt.x;
    float y8 = quad1->corners[(corner1+3)%4]->pt.y;

    // MARTIN: Heuristic
    // For corner2 of quad2 to be considered,
    // it needs to be on the other side of the two lines than
    // corner1. This is given, if the cross product has
    // a different sign for both computations below:
    float a5 = x6 - x5;
    float b5 = y6 - y5;
    // the current corner
    float c51 = quad1->corners[(corner1+2)%4]->pt.x - x5;
    float d51 = quad1->corners[(corner1+2)%4]->pt.y - y5;
    // the candidate corner
    float c52 = quad2->corners[corner2]->pt.x - x5;
    float d52 = quad2->corners[corner2]->pt.y - y5;
    float sign51 = a5*d51 - c51*b5;
    float sign52 = a5*d52 - c52*b5;

    float a6 = x8 - x7;
    float b6 = y8 - y7;
    // the current corner
    float c61 = quad1->corners[(corner1+2)%4]->pt.x - x7;
    float d61 = quad1->corners[(corner1+2)%4]->pt.y - y7;
    // the candidate corner
    float c62 = quad2->corners[corner2]->pt.x - x7;
    float d62 = quad2->corners[corner2]->pt.y - y7;
    float sign61 = a6*d61 - c61*b6;
    float sign62 = a6*d62 - c62*b6;


    // Fourth: Then check everything from the viewpoint of
    // the candidate quad compute midpoints of "parallel"
    // quad sides 1
    float u5 = quad2->corners[corner2]->pt.x;
    float v5 = quad2->corners[corner2]->pt.y;
    float u6 = quad2->corners[(corner2+1)%4]->pt.x;
    float v6 = quad2->corners[(corner2+1)%4]->pt.y;
    // compute midpoints of "parallel" quad sides 2
    float u7 = u5;
    float v7 = v5;
    float u8 = quad2->corners[(corner2+3)%4]->pt.x;
    float v8 = quad2->corners[(corner2+3)%4]->pt.y;

    // MARTIN: Heuristic
    // For corner2 of quad2 to be considered,
    // it needs to be on the other side of the two lines than
    // corner1. This is given, if the cross product has
    // a different sign for both computations below:
    float a7 = u6 - u5;
    float b7 = v6 - v5;
    // the current corner
    float c71 = quad1->corners[corner1]->pt.x - u5;
    float d71 = quad1->corners[corner1]->pt.y - v5;
    // the candidate corner
    float c72 = quad2->corners[(corner2+2)%4]->pt.x - u5;
    float d72 = quad2->corners[(corner2+2)%4]->pt.y - v5;
    float sign71 = a7*d71-c71*b7;
    float sign72 = a7*d72-c72*b7;

    float a8 = u8 - u7;
    float b8 = v8 - v7;
    // the current corner
    float c81 = quad1->corners[corner1]->pt.x - u7;
    float d81 = quad1->corners[corner1]->pt.y - v7;
    // the candidate corner
    float c82 = quad2->corners[(corner2+2)%4]->pt.x - u7;
    float d82 = quad2->corners[(corner2+2)%4]->pt.y - v7;
    float sign81 = a8*d81-c81*b8;
    float sign82 = a8*d82-c82*b8;

    // Check whether conditions are fulfilled
    if (((sign11 < 0 && sign12 < 0) || (sign11 > 0 && sign12 > 0))  &&
        ((sign21 < 0 && sign22 < 0) || (sign21 > 0 && sign22 > 0))  &&
        ((sign31 < 0 && sign32 < 0) || (sign31 > 0 && sign32 > 0))  &&
        ((sign41 < 0 && sign42 < 0) || (sign41 > 0 && sign42 > 0))  &&
        ((sign11 < 0 && sign13 < 0) || (sign11 > 0 && sign13 > 0))  &&
        ((sign21 < 0 && sign23 < 0) || (sign21 > 0 && sign23 > 0))  &&
        ((sign31 < 0 && sign33 < 0) || (sign31 > 0 && sign33 > 0))  &&
        ((sign41 < 0 && sign43 < 0) || (sign41 > 0 && sign43 > 0))  &&
        ((sign51 < 0 && sign52 > 0) || (sign51 > 0 && sign52 < 0))  &&
        ((sign61 < 0 && sign62 > 0) || (sign61 > 0 && sign62 < 0))  &&
        ((sign71 < 0 && sign72 > 0) || (sign71 > 0 && sign72 < 0))  &&
        ((sign81 < 0 && sign82 > 0) || (sign81 > 0 && sign82 < 0)))
    {
        return true;
    }
    else
    {
        return false;
    }
}

}
