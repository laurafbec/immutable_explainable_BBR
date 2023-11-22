import os
import yaml
import hashlib
import base64
from web3 import Web3
from ament_index_python.packages import get_package_share_directory
import math


class Blockchain:
    def __init__(self):
        # Load the blockchain configuration
        self.load_blockchain_config()

    def load_blockchain_config(self):
        # Get the absolute file path of the .env file
        self.blockchain_config_file = os.path.join(get_package_share_directory('tamper_proof_bag_recorder'), 'config', 'blockchain_config.yaml')

        # Load the YAML file
        with open(self.blockchain_config_file, 'r') as file:
            self.blockchain_config = yaml.safe_load(file)

        # Connect to the blockchain
        ganache_url = self.blockchain_config['ganache_url']
        self.web3 = Web3(Web3.HTTPProvider(ganache_url))

        # Get Blockchain values
        self.account_address = self.blockchain_config['account_address']
        self.account_private_key = self.blockchain_config['account_private_key']       
        self.contract_abi = self.blockchain_config['contract_abi']
        self.contract_address = self.blockchain_config['contract_address']
        self.contract_bytecode = self.blockchain_config['contract_bytecode']        


    def get_hash_value(self, message, previous_hash):
        # Encode the message string using base64
        encoded_message = base64.b64encode(message.encode('utf-8'))

        # Calculate the chain of hashes
        m = hashlib.sha256()
        m.update(encoded_message)
        m.update(previous_hash.encode('utf-8'))
        chained_hash = m.hexdigest()

        return chained_hash
        #return hash_value

    
    def build_and_send_transaction(self, contract, function_name, *args):
        # Get the nonce of the account
        nonce = self.web3.eth.get_transaction_count(self.account_address)

        if function_name is None:
            # Estimate the gas for contract deployment
            estimated_gas = contract.constructor().estimate_gas()
            # Build the transaction
            transaction = contract.constructor().build_transaction({
                'gas': int(estimated_gas * 1.2),
                'gasPrice': self.web3.eth.gas_price,
                'from': self.account_address,
                'nonce': nonce
            })
        else:
            # Estimate the gas fee
            function = getattr(contract.functions, function_name)
            estimated_gas = function(*args).estimate_gas({'from': self.account_address})
            #Build the transaction
            transaction = function(*args).build_transaction({
                'gas': int(estimated_gas * 1.2),
                'gasPrice': self.web3.eth.gas_price,
                'from': self.account_address,
                'nonce': nonce
            })

        # Sign the transaction
        signed_txn = self.web3.eth.account.sign_transaction(transaction, private_key=self.account_private_key)

        # Send the transaction
        tx_hash = self.web3.eth.send_raw_transaction(signed_txn.rawTransaction)

        # Wait for the transaction to confirm
        self.receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)

        return tx_hash
    
    def get_max_gas_price(self):
        # Get the latest block header
        latest_block = self.web3.eth.get_block('latest')
        next_gas_price = math.ceil(latest_block.get('baseFeePerGas') * 1.251)
        return next_gas_price
        
    def store_hash_in_blockchain(self, hash_value):
        # Load the contract
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)

        # Build and send the transaction
        self.build_and_send_transaction(contract, 'storeProofDocuments', hash_value)

    def check_proof_exists(self, hash_value):
        # Load the contract
        contract = self.web3.eth.contract(address=self.contract_address, abi=self.contract_abi)

        # Build and send the transaction
        tx_hash = self.build_and_send_transaction(contract, 'checkProofDocuments', hash_value)
        receipt = self.web3.eth.get_transaction_receipt(tx_hash)

        # Check if the document exists based on the emitted events
        events = contract.events.Result().process_receipt(receipt)

        block_number = 0

        if events:
            for event in events:
                if int(event["args"]["blockNumber"]) != 0:
                    block_number = int(event["args"]["blockNumber"])
                    break

        return block_number


    def deploy_contract(self):
        # Load the contract
        contract = self.web3.eth.contract(abi=self.contract_abi, bytecode=self.contract_bytecode)

        # Build and send the transaction
        tx_hash = self.build_and_send_transaction(contract, None)
        receipt = self.web3.eth.get_transaction_receipt(tx_hash)
        print(receipt)
        contract_address = receipt['contractAddress']
        print(f"Contract Address: {contract_address}")

        # Store contract address in an environment variable
        contract_address = receipt['contractAddress']
        self.update_contract_address(contract_address)

    def update_contract_address(self, contract_address):
        # Get the contract address
        self.blockchain_config['contract_address'] = contract_address

        # Update the blockchain-config.yaml file in the install directory
        #print(self.blockchain_config_file)
        with open(self.blockchain_config_file, 'w') as file:
            yaml.safe_dump(self.blockchain_config, file)
            

    

