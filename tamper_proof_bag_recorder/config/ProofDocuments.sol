// SPDX-License-Identifier: MIT
pragma solidity ^0.8;
contract ProofDocumentsStore {

// Hash of the documents and their corresponding block number.
mapping (bytes32 => uint) private proofs;

// Owner of the contract
address owner = msg.sender;

// Result containing the from, hash and blocknumber data.
event Result(
    address from,
    string document,
    uint blockNumber
);

function storeProofDocuments(string calldata documentHash) external {
        require(msg.sender == owner, "Only the owner of the contract can store the documents");
        bytes32 hashBytes = bytes32(bytes(documentHash));
        proofs[hashBytes] = block.number;
    }

    function checkProofDocuments(string calldata documentHash) external {
        bytes32 hashBytes = bytes32(bytes(documentHash));
        emit Result(msg.sender, documentHash, proofs[hashBytes]);
    }


}