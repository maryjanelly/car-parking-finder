import cv2
import numpy as np
import pickle
from src.utils import Park_classifier
from src.pathfinding.a_star import AStar  # Assuming we add an A* implementation in src/pathfinding

def find_nearest_empty_space(park_status, start_pos):
    """Finds the nearest empty space using A* pathfinding."""
    astar = AStar(park_status)
    empty_spaces = [(x, y) for x, row in enumerate(park_status) for y, status in enumerate(row) if status == 0]
    
    if not empty_spaces:
        return None
    
    paths = [(space, astar.find_path(start_pos, space)) for space in empty_spaces]
    paths = [p for p in paths if p[1] is not None]  # Filter inaccessible spots
    paths.sort(key=lambda p: len(p[1]))  # Shortest path
    
    return paths[0] if paths else None

def demostration():
     # defining the params
    rect_width, rect_height = 107, 48
    carp_park_positions_path = "data/source/CarParkPos"
    video_path = "data/source/carPark.mp4"
    # creating the classifier  instance which uses basic image processes to classify
    classifier = Park_classifier(carp_park_positions_path, rect_width, rect_height)
    # Implementation of the classifier
    cap = cv2.VideoCapture(video_path)
    
    while True:
        # reading the video frame by frame
        ret, frame = cap.read()
        if not ret:
            break

        # prosessing the frames to prepare classify
        prosessed_frame = classifier.implement_process(frame)

        # drawing car parks according to its status 
        denoted_image, park_status = classifier.classify(image=frame, prosessed_image=prosessed_frame, return_status=True)
        
        # Define start position and determine nearest empty space path
        start_pos = (0, 0)  # Define actual starting position
        nearest_space = find_nearest_empty_space(park_status, start_pos)
        
        if nearest_space:
            destination, path = nearest_space
            for x, y in path:  # Visualize path
                cv2.circle(denoted_image, (x, y), 5, (255, 0, 0), -1)
        
        # displaying the results
        cv2.imshow("Car Park Image with Paths to Empty Spots", denoted_image)
        
        k = cv2.waitKey(1)
        if k & 0xFF == ord('q'):
            break
        if k & 0xFF == ord('s'):
            cv2.imwrite("output.jpg", denoted_image)

    # re-allocating sources    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    demostration()
