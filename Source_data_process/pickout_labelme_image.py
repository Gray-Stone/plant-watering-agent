import argparse
import pathlib



if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument("image_dir", default='./data', type=pathlib.Path, help="output dir")
    parser.add_argument("output_dir", default='./selected', type=pathlib.Path, help="output dir")

    args = parser.parse_args()

    out_dir = args.output_dir

    image_source :pathlib.Path = args.image_dir

    if not image_source.exists():
        raise ValueError(f"Image dir {image_source} doesn't exists")
