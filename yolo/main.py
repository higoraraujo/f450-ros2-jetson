import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os

class AlgaeDetectionNode(Node):
    def __init__(self):
        super().__init__('algae_detection_node')
        
        # Subscrição à câmera (deve estar publicando imagens no formato de sensor_msgs/Image)
        self.image_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 10)
        
        # Inicializando a ponte do OpenCV com ROS
        self.bridge = CvBridge()

        # Carregando o YOLO
        package_share_directory = get_package_share_directory('algae_detection_package')
        weights_path = os.path.join(package_share_directory, 'yolo', 'yolov3.weights')
        config_path = os.path.join(package_share_directory, 'yolo', 'yolov3.cfg')
        names_path = os.path.join(package_share_directory, 'yolo', 'coco.names')
        
        self.net = cv2.dnn.readNet(weights_path, config_path)
        self.layer_names = self.net.getLayerNames()
        self.output_layers = [self.layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        
        self.classes = []
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

    def image_callback(self, msg):
        # Converter a imagem do ROS (sensor_msgs/Image) para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Processar a imagem com YOLO
        height, width, channels = cv_image.shape
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        # Variáveis para armazenar dados detectados
        class_ids = []
        confidences = []
        boxes = []

        # Processamento das saídas da YOLO
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and self.classes[class_id] == "algae":  # Detecta a classe "algae"
                    # Obter coordenadas da caixa delimitadora
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    # Coordenadas da caixa
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Aplicar Non-Maximum Suppression para eliminar caixas sobrepostas
        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Desenhar as caixas de detecção na imagem
        if len(indexes) > 0:
            for i in indexes.flatten():
                x, y, w, h = boxes[i]
                label = str(self.classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)  # Verde para as algas detectadas
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(cv_image, f"{label} {round(confidence, 2)}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Exibir a imagem processada
        cv2.imshow('Algae Detection', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = AlgaeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
