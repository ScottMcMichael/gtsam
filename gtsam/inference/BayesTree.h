/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    BayesTree.h
 * @brief   Bayes Tree is a tree of cliques of a Bayes Chain
 * @author  Frank Dellaert
 */

// \callgraph

#pragma once
#include <string>
#include <queue>
#include <boost/shared_ptr.hpp>

#include <gtsam/inference/Key.h>
#include <gtsam/base/FastList.h>
#include <gtsam/base/ConcurrentMap.h>
#include <gtsam/base/FastVector.h>



namespace gtsam {

  // Forward declarations
  template<class FACTOR> class FactorGraph;
  template<class BAYESTREE, class GRAPH> class EliminatableClusterTree;

  /* ************************************************************************* */
  /** clique statistics */
  struct GTSAM_EXPORT BayesTreeCliqueStats {
    double avgConditionalSize;
    std::size_t maxConditionalSize;
    double avgSeparatorSize;
    std::size_t maxSeparatorSize;
    void print(const std::string& s = "") const ;
  };

  /** store all the sizes  */
  struct GTSAM_EXPORT BayesTreeCliqueData {
    FastVector<std::size_t> conditionalSizes;
    FastVector<std::size_t> separatorSizes;
    BayesTreeCliqueStats getStats() const;
  };

  /* ************************************************************************* */
  /**
   * Bayes tree
   * @tparam CONDITIONAL The type of the conditional densities, i.e. the type of node in the underlying Bayes chain,
   * which could be a ConditionalProbabilityTable, a GaussianConditional, or a SymbolicConditional.
   * @tparam CLIQUE The type of the clique data structure, defaults to BayesTreeClique, normally do not change this
   * as it is only used when developing special versions of BayesTree, e.g. for ISAM2.
   *
   * \ingroup Multifrontal
   * \nosubgrouping
   */
  template<class CLIQUE>
  class BayesTree
  {
  protected:
    typedef BayesTree<CLIQUE> This;
    typedef boost::shared_ptr<This> shared_ptr;

  public:
    typedef CLIQUE Clique; ///< The clique type, normally BayesTreeClique
    typedef boost::shared_ptr<Clique> sharedClique; ///< Shared pointer to a clique
    typedef Clique Node; ///< Synonym for Clique (TODO: remove)
    typedef sharedClique sharedNode; ///< Synonym for sharedClique (TODO: remove)
    typedef typename CLIQUE::ConditionalType ConditionalType;
    typedef boost::shared_ptr<ConditionalType> sharedConditional;
    typedef typename CLIQUE::BayesNetType BayesNetType;
    typedef boost::shared_ptr<BayesNetType> sharedBayesNet;
    typedef typename CLIQUE::FactorType FactorType;
    typedef boost::shared_ptr<FactorType> sharedFactor;
    typedef typename CLIQUE::FactorGraphType FactorGraphType;
    typedef boost::shared_ptr<FactorGraphType> sharedFactorGraph;
    typedef typename FactorGraphType::Eliminate Eliminate;
    typedef typename CLIQUE::EliminationTraitsType EliminationTraitsType;

    /** A convenience class for a list of shared cliques */
    typedef FastList<sharedClique> Cliques;

    /** Map from keys to Clique */
    typedef ConcurrentMap<Key, sharedClique> Nodes;

    /** Root cliques */
    typedef FastVector<sharedClique> Roots;

  protected:

    /** Map from indices to Clique */
    Nodes nodes_;

    /** Root cliques */
    Roots roots_;

    /// @name Standard Constructors
    /// @{

    /** Create an empty Bayes Tree */
    BayesTree() {}

    /** Copy constructor */
    BayesTree(const This& other);

    /// @}

    /** Assignment operator */
    This& operator=(const This& other);

    /// @name Testable
    /// @{

    /** check equality */
    bool equals(const This& other, double tol = 1e-9) const;

  public:
    /** print */
    void print(const std::string& s = "",
        const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
    /// @}

    /// @name Standard Interface
    /// @{

    /** number of cliques */
    size_t size() const;

    /** Check if there are any cliques in the tree */
    inline bool empty() const {
      return nodes_.empty();
    }

    /** Return nodes. Each node is a clique of variables obtained after elimination. */
    const Nodes& nodes() const { return nodes_; }

    /** Access node by variable */
    sharedClique operator[](Key j) const { return nodes_.at(j); }

    /** return root cliques */
    const Roots& roots() const { return roots_;  }

    /** alternate syntax for matlab: find the clique that contains the variable with Key j */
    const sharedClique& clique(Key j) const {
      typename Nodes::const_iterator c = nodes_.find(j);
      if(c == nodes_.end())
        throw std::out_of_range("Requested the BayesTree clique for a key that is not in the BayesTree");
      else
        return c->second;
    }

    /** Gather data on all cliques */
    BayesTreeCliqueData getCliqueData() const;

    /** Collect number of cliques with cached separator marginals */
    size_t numCachedSeparatorMarginals() const;

    /** Return marginal on any variable.  Note that this actually returns a conditional, for which a
     *  solution may be directly obtained by calling .solve() on the returned object.
     *  Alternatively, it may be directly used as its factor base class.  For example, for Gaussian
     *  systems, this returns a GaussianConditional, which inherits from JacobianFactor and
     *  GaussianFactor. */
    sharedConditional marginalFactor(Key j, const Eliminate& function = EliminationTraitsType::DefaultEliminate) const;

    /**
     * return joint on two variables
     * Limitation: can only calculate joint if cliques are disjoint or one of them is root
     */
    sharedFactorGraph joint(Key j1, Key j2, const Eliminate& function = EliminationTraitsType::DefaultEliminate) const;

    /**
     * return joint on two variables as a BayesNet
     * Limitation: can only calculate joint if cliques are disjoint or one of them is root
     */
    sharedBayesNet jointBayesNet(Key j1, Key j2, const Eliminate& function = EliminationTraitsType::DefaultEliminate) const;

   /// @name Graph Display
   /// @{

   /// Output to graphviz format, stream version.
   void dot(std::ostream& os, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

   /// Output to graphviz format string.
   std::string dot(
       const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;

   /// output to file with graphviz format.
   void saveGraph(const std::string& filename,
                  const KeyFormatter& keyFormatter = DefaultKeyFormatter) const;
  
    /// @}
    /// @name Advanced Interface
    /// @{

    /**
     * Find parent clique of a conditional.  It will look at all parents and
     * return the one with the lowest index in the ordering.
     */
    template<class CONTAINER>
    Key findParentClique(const CONTAINER& parents) const;

    /** Remove all nodes */
    void clear();

    /** Clear all shortcut caches - use before timing on marginal calculation to avoid residual cache data */
    void deleteCachedShortcuts();

    /**
     * Remove path from clique to root and return that path as factors
     * plus a list of orphaned subtree roots. Used in removeTop below.
     */
    void removePath(sharedClique clique, BayesNetType* bn, Cliques* orphans);

    /**
     * Given a list of indices, turn "contaminated" part of the tree back into a factor graph.
     * Factors and orphans are added to the in/out arguments.
     */
    void removeTop(const KeyVector& keys, BayesNetType* bn, Cliques* orphans);

    /**
     * Remove the requested subtree. */
    Cliques removeSubtree(const sharedClique& subtree);

    /** Insert a new subtree with known parent clique.  This function does not check that the
     *  specified parent is the correct parent.  This function updates all of the internal data
     *  structures associated with adding a subtree, such as populating the nodes index. */
    void insertRoot(const sharedClique& subtree);

    /** add a clique (top down) */
    void addClique(const sharedClique& clique, const sharedClique& parent_clique = sharedClique());

    /** Add all cliques in this BayesTree to the specified factor graph */
    void addFactorsToGraph(FactorGraph<FactorType>* graph) const;

  protected:

    /** private helper method for saving the Tree to a text file in GraphViz format */
    void dot(std::ostream &s, sharedClique clique, const KeyFormatter& keyFormatter,
             int parentnum = 0) const;

    /** Gather data on a single clique */
    void getCliqueData(sharedClique clique, BayesTreeCliqueData* stats) const;

    /** remove a clique: warning, can result in a forest */
    void removeClique(sharedClique clique);

    /** Fill the nodes index for a subtree */
    void fillNodesIndex(const sharedClique& subtree);

    // Friend JunctionTree because it directly fills roots and nodes index.
    template<class BAYESTREE, class GRAPH> friend class EliminatableClusterTree;

   private:
       
       
    /** Serialization function */
    /*
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int /*version*//*) {
      std::cout << "BayesTree Serialize nodes_ with size: " << nodes_.size() << std::endl;
      

      auto iter = nodes_.begin();
      while (iter != nodes_.end()) {      
        std::cout << "This node tree size = " << iter->second->treeSize() << std::endl;
        std::cout << "This node tree is root? = " << iter->second->isRoot() << std::endl;
        
        Key k = iter->first;
        sharedClique cliquePtr = iter->second;
        
        cliquePtr->print();
        
          
        ar & BOOST_SERIALIZATION_NVP(k);
        //ar & BOOST_SERIALIZATION_NVP(cliquePtr);
        saveRoot(ar, cliquePtr);
        std::cout << "Done serializing node tree of  size = " << iter->second->treeSize() << std::endl;
        ++iter;
      }
      //ar & BOOST_SERIALIZATION_NVP(nodes_); // <-- crash route!
      std::cout << "BayesTree done serializing nodes_ with size  " << nodes_.size() << std::endl;


      std::cout << "BayesTree Serialize roots_ with size: " << roots_.size() << std::endl;
      ar & BOOST_SERIALIZATION_NVP(roots_);
    }
*/
    
    class PointerIndexMap {
      std::map<sharedClique, size_t> data;
       size_t counter;
    public:
        PointerIndexMap() : counter(0) {}
      size_t getIndex(sharedClique ptr){
        try {
          return data.at(ptr);
        }
        catch(std::out_of_range&){
          data[ptr] = counter;
//          std::cout << "Assigned index " << counter << std::endl;
          return counter++;
        }
      }
      
      size_t size() const {return data.size();}
      
      size_t hasData(sharedClique ptr){
        try {
          size_t dummy = data.at(ptr);
          return true;
        }
        catch(std::out_of_range&){
          return false;
        }
      }
      
      /// Create ordered vector of clique pointers
      void makeCliqueVector(std::vector<sharedClique> &v) {
        v.resize(size());
        for (std::pair<sharedClique, size_t> it : data) {
          v[it.second] = it.first;
        }
      }
    };
    
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void save(ARCHIVE & ar, const unsigned int /*version*/) const {
      std::cout << "------ Save start." << std::endl;
    
      std::cout << "Roots size = " << roots_.size() << std::endl;
      std::cout << "Nodes size = " << nodes_.size() << std::endl;
        
      // Load all cliques into the map
      PointerIndexMap allCliques;
      for (sharedClique ptr : roots_) {
        if (!ptr->isRoot()) {
          std::cout << "Found non-root in in roots_!!!!" << std::endl;
        }
        // Parse the entire tree starting from each root
        while (!ptr->isRoot()) {
          ptr = ptr->parent();
        }
        std::queue<sharedClique> q;
        q.push(ptr);
        while (!q.empty()) {
          ptr = q.front();
          q.pop();
          allCliques.getIndex(ptr);
          for (sharedClique c : ptr->children) {
            q.push(c);
          }
        }
        std::cout << "After this root, have " << allCliques.size() << " unique nodes." << std::endl;
        
        
      }
      std::cout << "Found " << allCliques.size() << " unique nodes in roots_." << std::endl;

      for (std::pair<Key, sharedClique> p : nodes_) {
        // Parse the entire tree, but quit if we encounter a familiar node

        // Move to the root of the tree
        sharedClique ptr = p.second;
        while (!ptr->isRoot()) {
          if (allCliques.hasData(ptr)) {
            //std::cout << "Breaking out early with found node" << std::endl;
            break;
          }
          ptr = ptr->parent();
        }
        if (allCliques.hasData(ptr)) {
          // Since we exhaustively searched the roots trees earlier, if we see a repeat here
          // then this should all be previously searched nodes.
          continue;
        }
        std::queue<sharedClique> q;
        q.push(ptr);
        while (!q.empty()) {
          ptr = q.front();
          if (allCliques.hasData(ptr)) {
            //std::cout << "Breaking out early with found node" << std::endl;
            break;
          }
          q.pop();
          allCliques.getIndex(ptr);
          for (sharedClique c : ptr->children) {
            q.push(c);
          }
        }
        std::cout << "After this node, have " << allCliques.size() << " unique nodes." << std::endl;
      }
      size_t numCliques = allCliques.size();
      std::cout << "Total number of cliques: " << allCliques.size() << std::endl;
      
      // Record sorted cliques
      std::vector<sharedClique> orderedCliques;
      allCliques.makeCliqueVector(orderedCliques);
      ar & BOOST_SERIALIZATION_NVP(numCliques);
      for (sharedClique ptr : orderedCliques) {
        bool isRoot = ptr->isRoot();
        ptr->is_root = isRoot;
        //std::cout << "isRoot = " << isRoot << std::endl;
        ptr->serializeNoLinks(ar);
        if (!isRoot) {
          size_t index = allCliques.getIndex(ptr->parent());
//          std::cout << "root index = " << index << std::endl;
          ar & BOOST_SERIALIZATION_NVP(index);
        }
        size_t numChildren = ptr->children.size();
        ar & BOOST_SERIALIZATION_NVP(numChildren);
        for (sharedClique c : ptr->children) {
          size_t index = allCliques.getIndex(c);
//          std::cout << "child index = " << index << std::endl;
          ar & BOOST_SERIALIZATION_NVP(index);
        }
      }
      std::cout << "Done recording sorted cliques." << std::endl;
      
      // Record nodes_
      // - Instead of pointers, record indices in the sorted clique vector
      size_t numNodes = nodes_.size();
      ar & BOOST_SERIALIZATION_NVP(numNodes);
      for (std::pair<Key, sharedClique> p : nodes_) {
        Key k = p.first;
        ar & BOOST_SERIALIZATION_NVP(k);
        size_t index = allCliques.getIndex(p.second);
        //std::cout << "node index = " << index << std::endl;
        ar & BOOST_SERIALIZATION_NVP(index);
        // Just record the node indices here, no need to go through the tree
/*
        // Move to the root of the tree
        sharedClique ptr = p.second;
        while (!ptr->isRoot()) {
          ptr = ptr->parent();
        }
        std::queue<sharedClique> q;
        q.push(ptr);
        while (!q.empty()) {
          ptr = q.front();
          q.pop();
          ptr->serializeNoLinks(ar);
          if (!ptr->isRoot()) {
            size_t index = allCliques.getIndex(ptr->parent());
            ar & BOOST_SERIALIZATION_NVP(index);
          }
          ar & BOOST_SERIALIZATION_NVP(ptr->children.size());
          for (sharedClique c : ptr->children) {
            size_t index = allCliques.getIndex(c);
            ar & BOOST_SERIALIZATION_NVP(index);
            q.push(c);
          }
        }
*/
      }
      std::cout << "Done recording nodes_." << std::endl;
      
      // Record roots_
      size_t numRoots = roots_.size();
      ar & BOOST_SERIALIZATION_NVP(numRoots);
      for (sharedClique ptr : roots_) {
        size_t index = allCliques.getIndex(ptr);
        std::cout << "root index = " << index << std::endl;
        ar & BOOST_SERIALIZATION_NVP(index);
      }
      std::cout << "Done recording roots_." << std::endl;
    }
    
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void load(ARCHIVE & ar, const unsigned int /*version*/) {
      std::cout << "------- Load start." << std::endl;
      
      nodes_.clear();
      roots_.clear();
      
      // First create all of the ordered cliques in memory
      size_t numCliques;
      ar & BOOST_SERIALIZATION_NVP(numCliques);
      std::vector<sharedClique> orderedCliques(numCliques);
      for (sharedClique &ptr : orderedCliques) {
        ptr.reset(new Clique);
      }
      std::cout << "Done creating " << numCliques << " cliques." << std::endl;

      // Now initialize the ordered cliques, including links between them
      // - Links are specified by the index number of the ordered clique vector
      size_t numChildren, index;
      for (sharedClique &ptr : orderedCliques) {
        ptr->serializeNoLinks(ar);
        std::cout << "ptr->is_root = " << ptr->is_root << std::endl;
        if (!ptr->is_root) {
          ar & BOOST_SERIALIZATION_NVP(index);
          std::cout << "root index = " << index << std::endl;
          ptr->parent_ = orderedCliques[index];
        }
        ar & BOOST_SERIALIZATION_NVP(numChildren);
        std::cout << "numChildren = " << numChildren << std::endl;
        ptr->children.resize(numChildren);
        for (size_t i=0; i<numChildren; ++i) {
          ar & BOOST_SERIALIZATION_NVP(index);
          std::cout << "child index = " << index << std::endl;
          ptr->children[i] = orderedCliques[index];
        }
      }
      std::cout << "Done linking initial cliques." << std::endl;
      
      // Populate nodes_
      size_t numNodes;
      ar & BOOST_SERIALIZATION_NVP(numNodes);
      std::cout << "numNodes: " << numNodes << std::endl;
      for (size_t i=0; i<numNodes; ++i) {
        Key k;
        ar & BOOST_SERIALIZATION_NVP(k);
        ar & BOOST_SERIALIZATION_NVP(index);
        std::cout << "key = " << k << std::endl;
        std::cout << "index = " << index << std::endl;
        nodes_[k] = orderedCliques[index];
      }
      std::cout << "Done loading nodes." << std::endl;
      
      // Populate roots_
      size_t numRoots;
      ar & BOOST_SERIALIZATION_NVP(numRoots);
      std::cout << "numRoots: " << numRoots << std::endl;
      roots_.resize(numRoots);
      for (size_t i=0; i<numRoots; ++i) {
        ar & BOOST_SERIALIZATION_NVP(index);
        std::cout << "index = " << index << std::endl;
        roots_[i] = orderedCliques[index];
      }
      std::cout << "Done loading roots." << std::endl;
    }
    BOOST_SERIALIZATION_SPLIT_MEMBER()
    
    /*
    
    template<class ARCHIVE>
    void saveRoot1(ARCHIVE & ar, sharedClique cliquePtr) {
        
      std::stack<sharedClique> s;
      sharedClique current = cliquePtr;
      bool wasPop = false;
      while(true) {
        std::cout << "wasPop =  " << wasPop << std::endl;
        current->print();
        if (wasPop) {
          current->serializeFinal(ar);
          if (s.empty()) {
            break;
          }
          current = s.top();
          s.pop();
          wasPop = true;
          continue;
        }
        current->serializeInitial(ar);
        bool is_root = false;
        sharedClique parent = current->parent_.lock();
        if (!parent) {
            is_root = true;
             std::cout << "hit root"<< std::endl;
        }
        if (!is_root) {
            s.push(current);
            current = parent;
            wasPop = false;
        } else {
          // If this is root, it will be finished in the next loop cycle
          wasPop = true;
        }
      }
    }
    

    */
    
    /// @}

  }; // BayesTree

  /* ************************************************************************* */
  template <class CLIQUE, typename = void>
  class BayesTreeOrphanWrapper : public CLIQUE::ConditionalType {
   public:
    typedef CLIQUE CliqueType;
    typedef typename CLIQUE::ConditionalType Base;

    boost::shared_ptr<CliqueType> clique;

    /**
     * @brief Construct a new Bayes Tree Orphan Wrapper object
     *
     * This object stores parent keys in our base type factor so that
     * eliminating those parent keys will pull this subtree into the
     * elimination.
     *
     * @param clique Orphan clique to add for further consideration in
     * elimination.
     */
    BayesTreeOrphanWrapper(const boost::shared_ptr<CliqueType>& clique)
        : clique(clique) {
      this->keys_.assign(clique->conditional()->beginParents(),
                         clique->conditional()->endParents());
    }

    void print(
        const std::string& s = "",
        const KeyFormatter& formatter = DefaultKeyFormatter) const override {
      clique->print(s + "stored clique", formatter);
    }
  };

} /// namespace gtsam
