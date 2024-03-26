from decouple import config


def check_intermediate_representation():
    list1 = []
    if not eval(config("IMAGE_REQUIRED")):
        return list1
    else:
        if eval(config("HISTOGRAM_IMAGE_FLAG")):
            histogram_path = config("IMAGE_PATH") + config("HISTOGRAM_IMAGE")
            list1.append(histogram_path)
        if eval(config("RAW_IMAGE_FLAG")):
            raw_path = config("IMAGE_PATH") + config("RAW_IMAGE")
            list1.append(raw_path)
        if eval(config("DISPARITY_IMAGE_FLAG")):
            disparity_path = config("IMAGE_PATH") + config("DISPARITY_IMAGE")
            list1.append(disparity_path)
        return list1


# print(check_intermediate_representation())
# returned_val = check_intermediate_representation()
# if returned_val[0] == "False":
#     print("Got flase")
#
