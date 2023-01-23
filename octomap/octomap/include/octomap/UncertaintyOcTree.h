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

#ifndef OCTOMAP_UNCERTAINTY_OCTREE_H
#define OCTOMAP_UNCERTAINTY_OCTREE_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <iostream>

namespace octomap {

    // Forward declaration for "friend"
    class UncertaintyOcTree;

    // Node definition
    class UncertaintyOcTreeNode : public OcTreeNode {

    public:
        // file I/O
        std::istream &readData(std::istream &s);

        std::ostream &writeData(std::ostream &s) const;

        friend class ColorOcTree; // needs access to node children (inherited)

        class Color {
        public:
            Color() : r(255), g(255), b(255) {}

            Color(uint8_t _r, uint8_t _g, uint8_t _b)
                    : r(_r), g(_g), b(_b) {}

            inline bool operator==(const Color &other) const {
                return (r == other.r && g == other.g && b == other.b);
            }

            inline bool operator!=(const Color &other) const {
                return (r != other.r || g != other.g || b != other.b);
            }

            uint8_t r, g, b;
        };

        friend class UncertaintyOcTree;  // needs access to node children (inherited)

        UncertaintyOcTreeNode() : OcTreeNode(), uncertainty(1.0) {}

        UncertaintyOcTreeNode(const UncertaintyOcTreeNode &rhs) : OcTreeNode(rhs), uncertainty(rhs.uncertainty) {}

        bool operator==(const UncertaintyOcTreeNode &rhs) const {
            return (rhs.value == value && rhs.color == color && rhs.uncertainty == uncertainty);
        }

        void copyData(const UncertaintyOcTreeNode &from) {
            OcTreeNode::copyData(from);
            this->color = from.getColor();
            this->uncertainty = from.getUncertainty();
        }

        inline Color getColor() const { return color; }

        inline void setColor(Color c) { this->color = c; }

        inline void setColor(uint8_t r, uint8_t g, uint8_t b) {
            this->color = Color(r, g, b);
        }

        Color &getColor() { return color; }

        // has any color been integrated? (pure white is very unlikely...)
        inline bool isColorSet() const {
            return ((color.r != 255) || (color.g != 255) || (color.b != 255));
        }

        void updateColorChildren();


        UncertaintyOcTreeNode::Color getAverageChildColor() const;

        // Uncertainty
        inline float getUncertainty() const { return this->uncertainty; }

        inline void setUncertainty(float u) { this->uncertainty = u; }

        // Has any uncertainty been integrated?
        inline bool isUncertaintySet() const { return (this->uncertainty < 1.0); }

        float getLogitMeanChildUncertainty() const;

        float getMeanChildUncertainty() const;

        float getMaxChildUncertainty() const;

        float getMinChildUncertainty() const;

        void updateUncertaintyChildren();

    protected:

        float uncertainty;
        Color color;

    };


    // Tree definition
    class UncertaintyOcTree : public OccupancyOcTreeBase<UncertaintyOcTreeNode> {

    public:

        /// Default constructor, sets resolution of leafs
        UncertaintyOcTree(double resolution);

        /// virtual constructor: creates a new object of same type
        /// (Covariant return type requires an up-to-date compiler)
        UncertaintyOcTree *create() const { return new UncertaintyOcTree(resolution); }

        std::string getTreeType() const { return "UncertaintyOcTree"; }

        /**
         * Prunes a node when it is collapsible. This overloaded
         * version only considers the node occupancy for pruning,
         * different uncertainties of child nodes are ignored.
         * @return true if pruning was successful
         */
        virtual bool pruneNode(UncertaintyOcTreeNode *node);

        virtual bool isNodeCollapsible(const UncertaintyOcTreeNode *node) const;

        // set node color at given key or coordinate. Replaces previous color.
        UncertaintyOcTreeNode *setNodeColor(const OcTreeKey &key, uint8_t r,
                                            uint8_t g, uint8_t b);

        UncertaintyOcTreeNode *setNodeColor(float x, float y,
                                            float z, uint8_t r,
                                            uint8_t g, uint8_t b) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return setNodeColor(key, r, g, b);
        }

        // integrate color measurement at given key or coordinate. Average with previous color
        UncertaintyOcTreeNode *averageNodeColor(const OcTreeKey &key, uint8_t r,
                                                uint8_t g, uint8_t b);

        UncertaintyOcTreeNode *averageNodeColor(float x, float y,
                                                float z, uint8_t r,
                                                uint8_t g, uint8_t b) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return averageNodeColor(key, r, g, b);
        }

        // integrate color measurement at given key or coordinate. Average with previous color
        UncertaintyOcTreeNode *integrateNodeColor(const OcTreeKey &key, uint8_t r,
                                                  uint8_t g, uint8_t b);

        UncertaintyOcTreeNode *integrateNodeColor(float x, float y,
                                                  float z, uint8_t r,
                                                  uint8_t g, uint8_t b) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return integrateNodeColor(key, r, g, b);
        }

        // Get node uncertainty at given key or coordinate.
        float getNodeUncertainty(const OcTreeKey &key);

        float getNodeUncertainty(float x, float y, float z) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return 1.0;
            return getNodeUncertainty(key);
        }

        // Set node uncertainty at given key or coordinate. Replaces previous uncertainty.
        UncertaintyOcTreeNode *setNodeUncertainty(const OcTreeKey &key, float u);

        UncertaintyOcTreeNode *setNodeUncertainty(float x, float y, float z, float u) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return setNodeUncertainty(key, u);
        }

        // Mean uncertainty measurement at given key or coordinate with previous uncertainty
        UncertaintyOcTreeNode *meanNodeUncertainty(const OcTreeKey &key, float u);

        UncertaintyOcTreeNode *meanNodeUncertainty(float x, float y, float z, float u) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return meanNodeUncertainty(key, u);
        }

        // Integrate uncertainty measurement at given key or coordinate with previous uncertainty
        UncertaintyOcTreeNode *integrateNodeUncertainty(const OcTreeKey &key, float u);

        UncertaintyOcTreeNode *integrateNodeUncertainty(float x, float y, float z, float u) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return integrateNodeUncertainty(key, u);
        }

        // Mean uncertainty measurement at given key or coordinate with previous uncertainty
        UncertaintyOcTreeNode *minNodeUncertainty(const OcTreeKey &key, float u);

        UncertaintyOcTreeNode *minNodeUncertainty(float x, float y, float z, float u) {
            OcTreeKey key;
            if (!this->coordToKeyChecked(point3d(x, y, z), key)) return NULL;
            return minNodeUncertainty(key, u);
        }

        // Update inner nodes, sets uncertainty to mean child uncertainty
        void updateInnerOccupancy();

        float calculateUncertainty(point3d sensor_origin, point3d point_origin);

        float calculateUncertainty(point3d point_sensor);

        // uses gnuplot to plot a RGB histogram in EPS format
        void writeColorHistogram(std::string filename);


    protected:

        void updateInnerOccupancyRecurs(UncertaintyOcTreeNode *node, unsigned int depth);

        /**
         * Static member object which ensures that this OcTree's prototype
         * ends up in the classIDMapping only once. You need this as a
         * static member in any derived octree class in order to read .ot
         * files through the AbstractOcTree factory. You should also call
         * ensureLinking() once from the constructor.
         */
        class StaticMemberInitializer {
        public:

            StaticMemberInitializer() {
                UncertaintyOcTree *tree = new UncertaintyOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            /**
             * Dummy function to ensure that MSVC does not drop the
             * StaticMemberInitializer, causing this tree failing to register.
             * Needs to be called from the constructor of this octree.
             */
            void ensureLinking() {};

        };

        /// Static member to ensure static initialization (only once)
        static StaticMemberInitializer uncertaintyOcTreeMemberInit;

    };

    //! user friendly output in format (r g b)
    std::ostream &operator<<(std::ostream &out, UncertaintyOcTreeNode::Color const &c);

}  // namespace octomap

#endif
