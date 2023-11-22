// SPDX-License-Identifier: MIT
pragma solidity ^0.8;

contract ProofDocumentsStore {
    // Hash of the documents and their corresponding block number.
    mapping(bytes32 => uint) private proofs;

    // Owner of the contract
    address owner = msg.sender;

    // Result containing the from, hash, and block number data.
    event Result(address from, string document, uint blockNumber);

    function storeProofDocuments(string[] calldata documentHashes) external {
        require(msg.sender == owner, "Only the owner of the contract can store the documents");

        for (uint i = 0; i < documentHashes.length; i++) {
            bytes32 hashBytes = bytes32(bytes(documentHashes[i]));
            proofs[hashBytes] = block.number;
        }
    }

    function checkProofDocuments(string calldata documentHash) external {
        bytes32 hashBytes = bytes32(bytes(documentHash));
        emit Result(msg.sender, documentHash, proofs[hashBytes]);
    }
}