2024-05-01Z-05

# Achievements:

labelme2yolo conversion script forked and modded.

Workflow: 

anylabeling -> lableme format -> yolo format

FiftyOne downloading data is working.

---

Using pre-trained and extended model doing prediction

note: 
* Pretrained COCO is doing potted plants and vase. potted plant seems to have leaves in it. Vase seems to only do the vase.



# Questions:

---

Should I continue the route of using pre-trained and expand it?

---

Another option is to train a clean one on the full COCO and Open Images set combined, Every label or just the Pots?

---


How do deal with COCO included leaves, Open Images only did pot


---


# notes 

Bouding box is not enough, segmentation at least. and only the pot.

Actuation problem: What is the goal pose of the end effector


FUndimental question is: input data -> goal pose output.

Try the simplier route: 2d image segmentation, then for each pixel's 3d point, slowly build a probability octomap for where the pot is. Once we know a 3d point clouds of a pot, we should be good.