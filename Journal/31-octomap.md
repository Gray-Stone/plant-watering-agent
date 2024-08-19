## Setting cell free

To set all cell below certain z to free (useful when clearing floor on existing map)

```
    float min_clamp_log = octree_ptr->getClampingThresMinLog();
    for (auto it = octree_ptr->begin(), end = octree_ptr->end(); it != end; ++it) {
      // Get the z-coordinate of the node's center
      double z = it.getZ();

      // Check if the z-coordinate is below the threshold
      if (z < 0) {
        // Mark the node as free space
        it->setLogOdds(min_clamp_log);
      }
    }
    octree_ptr->updateInnerOccupancy();

```

Specially note: this line (given by chatgpt)
```
octree_ptr->updateNode(it.getKey(), false);
```
might look very appealing as it gives a binary true false input. This is only updating the node by a delta amount of log probability, not directly setting its probability. The turn/false is kinda a short hand to represent increment of a preset delta when hit and miss.

