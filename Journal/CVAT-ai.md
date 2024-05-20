

##

Trying CVAT again as anylabeling have some small bugs and not as good a UI.

## Mount local folder so CVAT can directly read image out of it.

https://docs.cvat.ai/docs/administration/advanced/installation_automatic_annotation/

The additional file `docker-compose.override.yml` need to be added. 

For a simple, exposing a local folder for cvat to read image from, only change the one line `device: /mnt/share` to local folder path is enough.

If made a mistake, need to purge the bad volumes from docker before
https://github.com/cvat-ai/cvat/issues/6924


## AI assist tools

Additional setups are needed to enable this feature. 

https://docs.cvat.ai/docs/administration/advanced/installation_automatic_annotation/

All instruction should be available under the document.

* need a new docker compose launch command with added components.
* nuclio is needed. The executable need to install is nuctl, which is a tool to link ML models for cvat.
* Special nvidia docker driver is needed if desire to run model with GPU.

Launch the new docker sets with 
```
docker compose -f docker-compose.yml -f components/serverless/docker-compose.serverless.yml up -d
```
Note: When doing this, since each docker compose file is manually specified, it is necessary to manually add the override file for mounting.

nuclio have its own dashboard default at `http://localhost:8070` 

After things are setup and installed, deploy a model into nuclio by using ready made script.

For example, using SAM on GPU: 
`./serverless/deploy_gpu.sh ./serverless/pytorch/facebookresearch/sam` 


After just installing nvidia container driver, a reboot is needed. or will get this error.
```
docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]].
```


## UFW problem

When using the SAM or any ai tool, might get a 500 error. Bascially cvat cannot communicate with the nuclio. In my case, this is caused by ufw, simply apt purge it works.


## Segmentation format

CVAT represents the segmentation in two different modes. Mask or polygone. When exporting to COCO, Mask will generate a different syntax, which is not handled by the conversion script used. Need to make sure convering it in CVAT