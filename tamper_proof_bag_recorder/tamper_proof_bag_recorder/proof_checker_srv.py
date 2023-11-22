from custom_interfaces.srv import ProofCheckerServiceMessage
import rclpy
from rclpy.node import Node
import re

from .blockchain import Blockchain

class Service(Node):

    def __init__(self):
        super().__init__('proof_checker')
        
        # Create the service
        self.srv = self.create_service(ProofCheckerServiceMessage, 'proof_checker', self.ProofCheckerService_callback)

        # Set the blockchain
        self.blockchain = Blockchain()   
        
      

    def ProofCheckerService_callback(self, request, response):
        # Check if the param is a hash value
        if self.is_valid_hash(request.proof):
            hash_value = request.proof
            self.blockchain.load_blockchain_config()
            block_number = self.blockchain.check_proof_exists(hash_value)
            if block_number != 0:
                response.answer = "The hash value is stored in the block number " + str(block_number) + " on the Blockchain network"
            else:
                response.answer = "The hash value is not stored on the Blockchain network"   
        # return the response parameter
        return response
    
    
    def is_valid_hash(self, string):
        pattern = r'\b[A-Fa-f0-9]{64}\b'  # Assuming SHA-256 hash is 64 characters long
        match = re.search(pattern, string)
        if match:
            hash_value = match.group()
            try:
                bytes.fromhex(hash_value)
                return True
            except ValueError:
                return False
        else:
            return False


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()