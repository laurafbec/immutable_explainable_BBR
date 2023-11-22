import rclpy
from rclpy.node import Node
from .blockchain import Blockchain

class BlockchainContractDeployment(Node):
    def __init__(self):

        #Init contract deployment node
        super().__init__('blockchain_contract_deployment_node')

        # Set the blockchain
        self.blockchain = Blockchain()
        self.blockchain.load_blockchain_config()

        # Contract deployment
        self.blockchain.deploy_contract()
    
     
def main(args=None):
    rclpy.init(args=args)
    blockchain_contract_node = BlockchainContractDeployment()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



