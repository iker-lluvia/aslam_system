/*
 * OctoMap - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
 * http://octomap.github.com/
 *
 * Copyright (c) 2009-2013, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap/UncertaintyOcTree.h>

#include <math.h>

namespace octomap {

    // Node implementation  -------------------------------------------------

    std::ostream &UncertaintyOcTreeNode::writeData(std::ostream &s) const {
        s.write((const char *) &value, sizeof(value)); // occupancy
        s.write((const char *) &color, sizeof(Color)); // color
        s.write((const char *) &uncertainty, sizeof(float)); // uncertainty

        return s;
    }


    std::istream &UncertaintyOcTreeNode::readData(std::istream &s) {
        s.read((char *) &value, sizeof(value)); // occupancy
        s.read((char *) &color, sizeof(Color)); // color
        s.read((char *) &uncertainty, sizeof(float)); // uncertainty

        return s;
    }

    UncertaintyOcTreeNode::Color UncertaintyOcTreeNode::getAverageChildColor() const {
        int mr = 0;
        int mg = 0;
        int mb = 0;
        int c = 0;

        if (children != NULL) {
            for (int i = 0; i < 8; i++) {
                UncertaintyOcTreeNode *child = static_cast<UncertaintyOcTreeNode *>(children[i]);

                if (child != NULL && child->isColorSet()) {
                    mr += child->getColor().r;
                    mg += child->getColor().g;
                    mb += child->getColor().b;
                    ++c;
                }
            }
        }

        if (c > 0) {
            mr /= c;
            mg /= c;
            mb /= c;
            return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
        } else { // no child had a color other than white
            return Color(255, 255, 255);
        }
    }


    void UncertaintyOcTreeNode::updateColorChildren() {
        color = getAverageChildColor();
    }

    float UncertaintyOcTreeNode::getLogitMeanChildUncertainty() const {
        float mean = 0.0;
        int counter = 0;

        if (children != NULL) {
            for (int i = 0; i < 8; ++i) {
                UncertaintyOcTreeNode *child = static_cast<UncertaintyOcTreeNode *>(children[i]);
                if (child != NULL) {
                    mean += child->getUncertainty();
                    ++counter;
                }
            }
        }

        if (counter > 0) {
            mean /= (float) counter;
        }
        return log(mean / (1 - mean));

    }


    float UncertaintyOcTreeNode::getMeanChildUncertainty() const {
        float mean = 0.0;
        int counter = 0;

        if (children != NULL) {
            for (int i = 0; i < 8; ++i) {
                UncertaintyOcTreeNode *child = static_cast<UncertaintyOcTreeNode *>(children[i]);
                if (child != NULL) {
                    mean += child->getUncertainty();
                    ++counter;
                }
            }
        }

        if (counter > 0) {
            mean /= (float) counter;
            return mean;
        } else // no child had a positive uncertainty
        {
            return 1.0;
        }
    }


    float UncertaintyOcTreeNode::getMaxChildUncertainty() const {
        float max = std::numeric_limits<float>::lowest();

        if (children != NULL) {
            for (int i = 0; i < 8; ++i) {
                UncertaintyOcTreeNode *child = static_cast<UncertaintyOcTreeNode *>(children[i]);
                if (child != NULL) {
                    float u = child->getUncertainty();
                    if (u > max) {
                        max = u;
                    }
                }
            }
        }

        return max;
    }


    float UncertaintyOcTreeNode::getMinChildUncertainty() const {
        float min = std::numeric_limits<float>::max();

        if (children != NULL) {
            for (int i = 0; i < 8; ++i) {
                UncertaintyOcTreeNode *child = static_cast<UncertaintyOcTreeNode *>(children[i]);
                if (child != NULL) {
                    float u = child->getUncertainty();
                    if (u < min) {
                        min = u;
                    }
                }
            }
        }

        return min;
    }


    void UncertaintyOcTreeNode::updateUncertaintyChildren() {
        setUncertainty(getMaxChildUncertainty());
    }


    // Tree implementation -------------------------------------------------

    UncertaintyOcTree::UncertaintyOcTree(double in_resolution)
            : OccupancyOcTreeBase<UncertaintyOcTreeNode>(in_resolution) {
        uncertaintyOcTreeMemberInit.ensureLinking();
    }


    bool UncertaintyOcTree::pruneNode(UncertaintyOcTreeNode *node) {
        if (!isNodeCollapsible(node))
            return false;

        // set value to children's values (all assumed equal)
        node->copyData(*(getNodeChild(node, 0)));

        if (node->isColorSet()) // TODO check
            node->setColor(node->getAverageChildColor());

        if (node->isUncertaintySet()) // TODO check
            node->setUncertainty(node->getMaxChildUncertainty());


        // delete children
        for (unsigned int i = 0; i < 8; ++i) {
            deleteNodeChild(node, i);
        }
        delete[] node->children;
        node->children = NULL;

        return true;
    }


    bool UncertaintyOcTree::isNodeCollapsible(const UncertaintyOcTreeNode *node) const {
        // all children must exist, must not have children of
        // their own and have the same occupancy probability
        if (!nodeChildExists(node, 0))
            return false;

        const UncertaintyOcTreeNode *firstChild = getNodeChild(node, 0);
        if (nodeHasChildren(firstChild))
            return false;

        for (unsigned int i = 1; i < 8; ++i) {
            // compare nodes only using their occupancy, ignoring uncertainty for pruning
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) ||
                !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
                return false;
        }

        return true;
    }

    UncertaintyOcTreeNode *UncertaintyOcTree::setNodeColor(const OcTreeKey &key,
                                                           uint8_t r,
                                                           uint8_t g,
                                                           uint8_t b) {
        UncertaintyOcTreeNode *n = search(key);
        if (n != 0) {
            n->setColor(r, g, b);
        }
        return n;
    }

    UncertaintyOcTreeNode *UncertaintyOcTree::averageNodeColor(const OcTreeKey &key,
                                                               uint8_t r,
                                                               uint8_t g,
                                                               uint8_t b) {
        UncertaintyOcTreeNode *n = search(key);
        if (n != 0) {
            if (n->isColorSet()) {
                UncertaintyOcTreeNode::Color prev_color = n->getColor();
                n->setColor((prev_color.r + r) / 2, (prev_color.g + g) / 2, (prev_color.b + b) / 2);
            } else {
                n->setColor(r, g, b);
            }
        }
        return n;
    }

    UncertaintyOcTreeNode *UncertaintyOcTree::integrateNodeColor(const OcTreeKey &key,
                                                                 uint8_t r,
                                                                 uint8_t g,
                                                                 uint8_t b) {
        UncertaintyOcTreeNode *n = search(key);
        if (n != 0) {
            if (n->isColorSet()) {
                UncertaintyOcTreeNode::Color prev_color = n->getColor();
                double node_prob = n->getOccupancy();
                uint8_t new_r = (uint8_t) ((double) prev_color.r * node_prob
                                           + (double) r * (0.99 - node_prob));
                uint8_t new_g = (uint8_t) ((double) prev_color.g * node_prob
                                           + (double) g * (0.99 - node_prob));
                uint8_t new_b = (uint8_t) ((double) prev_color.b * node_prob
                                           + (double) b * (0.99 - node_prob));
                n->setColor(new_r, new_g, new_b);
            } else {
                n->setColor(r, g, b);
            }
        }
        return n;
    }


    float UncertaintyOcTree::getNodeUncertainty(const OcTreeKey &key) {
        UncertaintyOcTreeNode *node = search(key);
        if (node != NULL) {
            if (node->isUncertaintySet()) {
                return node->getUncertainty();
            }
        }
        return 1.0;
    }


    UncertaintyOcTreeNode *UncertaintyOcTree::setNodeUncertainty(const OcTreeKey &key, float u) {
        UncertaintyOcTreeNode *node = search(key);
        if (node != NULL) {
            node->setUncertainty(u);
        }
        return node;
    }


    UncertaintyOcTreeNode *UncertaintyOcTree::meanNodeUncertainty(const OcTreeKey &key, float u) {
        UncertaintyOcTreeNode *node = search(key);
        if (node != 0) {
            if (node->isUncertaintySet()) {
                float prev_uncertainty = node->getUncertainty();
                node->setUncertainty((prev_uncertainty + u) / 2);
            } else {
                node->setUncertainty(u);
            }
        }
        return node;
    }


    UncertaintyOcTreeNode *UncertaintyOcTree::integrateNodeUncertainty(const OcTreeKey &key, float u) {
        UncertaintyOcTreeNode *node = search(key);
        if (node != 0) {
            if (node->isUncertaintySet()) {
                float prev_uncertainty = node->getUncertainty();
                float node_prob = node->getOccupancy();
                float new_u = prev_uncertainty * node_prob + u * (0.99 - node_prob);
                node->setUncertainty(new_u);
            } else {
                node->setUncertainty(u);
            }
        }
        return node;
    }


    UncertaintyOcTreeNode *UncertaintyOcTree::minNodeUncertainty(const OcTreeKey &key, float u) {
        UncertaintyOcTreeNode *node = search(key);
        if (node != 0) {
            if (node->isUncertaintySet()) {
                float prev_uncertainty = node->getUncertainty();
                if (prev_uncertainty > u) {
                    node->setUncertainty(u);
                }
            } else {
                node->setUncertainty(u);
            }
        }
        return node;
    }


    void UncertaintyOcTree::updateInnerOccupancy() {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }


    void UncertaintyOcTree::updateInnerOccupancyRecurs(UncertaintyOcTreeNode *node, unsigned int depth) {
        // only recurse and update for inner nodes
        if (nodeHasChildren(node)) {
            // return early for last level
            if (depth < this->tree_depth) {
                for (unsigned int i = 0; i < 8; ++i) {
                    if (nodeChildExists(node, i)) {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
                    }
                }
            }
            node->updateOccupancyChildren();
            node->updateUncertaintyChildren();
        }
    }

    void UncertaintyOcTree::writeColorHistogram(std::string filename) {

#ifdef _MSC_VER
        fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
        // build RGB histogram
        std::vector<int> histogram_r(256, 0);
        std::vector<int> histogram_g(256, 0);
        std::vector<int> histogram_b(256, 0);
        for (UncertaintyOcTree::tree_iterator it = this->begin_tree(),
                     end = this->end_tree(); it != end; ++it) {
            if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
            UncertaintyOcTreeNode::Color &c = it->getColor();
            ++histogram_r[c.r];
            ++histogram_g[c.g];
            ++histogram_b[c.b];
        }
        // plot data
        FILE *gui = popen("gnuplot ", "w");
        fprintf(gui, "set term postscript eps enhanced color\n");
        fprintf(gui, "set output \"%s\"\n", filename.c_str());
        fprintf(gui, "plot [-1:256] ");
        fprintf(gui, "'-' w filledcurve lt 1 lc 1 tit \"r\",");
        fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
        fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
        fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
        fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
        fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

        for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_r[i]);
        fprintf(gui, "0 0\n");
        fprintf(gui, "e\n");
        for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_g[i]);
        fprintf(gui, "0 0\n");
        fprintf(gui, "e\n");
        for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_b[i]);
        fprintf(gui, "0 0\n");
        fprintf(gui, "e\n");
        for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_r[i]);
        fprintf(gui, "e\n");
        for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_g[i]);
        fprintf(gui, "e\n");
        for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_b[i]);
        fprintf(gui, "e\n");
        fflush(gui);
#endif
    }

    std::ostream &operator<<(std::ostream &out, UncertaintyOcTreeNode::Color const &c) {
        return out << '(' << (unsigned int) c.r << ' ' << (unsigned int) c.g << ' ' << (unsigned int) c.b << ')';
    }


    //~ float UncertaintyOcTree::calculateUncertainty(point3d sensor_origin, point3d point_origin)
    //~ {
    //~ // Calculate the uncertainty using the distance
    //~ // Sigmoid function to calculate the uncertainty based on the distance, u = sigmoid(d) = |1 - 2/(1 + 2^d)|
    //~ // https://www.desmos.com/calculator/wqdjigjxxy
    //~ float optimal_distance = 1.;  // centre of depth of field
    //~ float dist = (point_origin - sensor_origin).norm();
    //~ float d = abs(distance - optimal_distance);
    //~ float u = (float) (1. - 2./(1. + pow(2, d)));  // U(dist) = 1 - 2/(1 + 2^d)

    //~ return u;
    //~ }


    float UncertaintyOcTree::calculateUncertainty(point3d point_sensor) {
        // Calculate the uncertainty using the distance and the angle
        // Sigmoid function to calculate the uncertainty based on the distance, u = sigmoid(d) = |1 - 2/(1 + 2^d)|
        // Angle uncertainty is calculated with trigonometry
        // https://www.desmos.com/calculator/vgvb3pkrcb
        float optimal_distance = 1.;  // centre of depth of field
        float distance = point_sensor.norm();
        float d = abs(distance - optimal_distance);
        float e = 2.71828;
        float u_distance = (float) (1. - 2. / (1. + pow(e, d)));  // U(dist) = 1 - 2/(1 + e^d)

        float sensor_horizontal_fov = 0.52;
        float sensor_vertical_fov = 0.41;
        float angle_h = atan(point_sensor.x() / point_sensor.z());  //  A(h) = atan(Px / Pz)
        float angle_v = atan(point_sensor.y() / point_sensor.z());  //  A(v) = atan(Py / Pz)
        float u_angle = (abs(angle_h / sensor_horizontal_fov) + abs(angle_v / sensor_vertical_fov)) / 2;  // U(angle) = (|A(h)/FOV(h)| + |A(v)/FOV(v)|) / 2

        float u = (u_distance + u_angle) / 2;

        return u;
    }


    UncertaintyOcTree::StaticMemberInitializer UncertaintyOcTree::uncertaintyOcTreeMemberInit;

}  // namespace octomap
