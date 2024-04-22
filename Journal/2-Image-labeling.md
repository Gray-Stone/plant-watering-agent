

## Ideas for labeling tool:

Idea of quick image labeling.

Label a few starting one with ML.

Then use the very bad model to help label images with human. ML will do a quick label of it, then human correct the labeling.

Then keep improving the trained model using labeled images.


This site might have this idea already implmented:

https://roboflow.com/auto-label


This is offline model

https://github.com/vietanhdev/anylabeling

## Problem on data source selection

Since watering the plant is really more of a finding the pot then finding the plant, the ML model should actually be learning the pot not the plant.

However the types of pot can very so much. Specially if the goal is to try finding the pot without having image of them in the training set at all. Not only are there planty of pot types, but also the question of vase. Then if we count the modern styled pots, which could just be concrete tubes


### Decision Needed:

What should the training set be containing? 
* Only pots, the rather classic styled: round shape, tapering inwards.
* Include modern pots and vase, like those concrete and glass tubes.
* Include every every thing, as long as it have plants in it, feed it to ML

## Labeling Choice: 

There are segmentation and detection difference for ML, and each need differently labeled dataset.

Also should the plant part be included in the label? Would the plant half be helpful on making the right decision about those glass tubes?

### Decision Needed: 

* Segmentation vs Detection (bounding box)
* Label include the plant or not