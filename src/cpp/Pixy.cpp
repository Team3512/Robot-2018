// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#include "Pixy.hpp"

#include <llvm/raw_ostream.h>

void Pixy::Block::Print() const {
    // Color code (CC)
    if (signature > kMaxSignature) {
        // Convert signature number to an octal string.
        char sig[6];
        int j = 0;
        bool flag = false;
        for (int i = 12; i >= 0; i -= 3) {
            char d = (signature >> i) & 0x07;
            if (d > 0 && !flag) {
                flag = true;
            }
            if (flag) {
                sig[j++] = d + '0';
            }
        }

        // Assigns value to signature, x, y, width, height, and angle.
        sig[j] = '\0';
        llvm::outs() << "CC block! sig: " << sig << " (" << signature
                     << " decimal) x: " << x << " y: " << y
                     << " width: " << width << " height: " << height
                     << " angle: " << angle << "\n";
    } else {
        // Regular block. Note, angle is always zero, so no need to print.
        llvm::outs() << "sig: " << signature << " x: " << x << " y: " << y
                     << " width: " << width << " height: " << height << "\n";
    }
}

bool Pixy::GetStart() {
    uint16_t lastw = 0xffff;

    while (true) {
        uint16_t w = GetWord();
        if (w == 0 && lastw == 0) {
            return false;
        } else if (w == kStartWord && lastw == kStartWord) {
            blockType = kNormalBlock;
            return true;
        } else if (w == kStartWordCc && lastw == kStartWord) {
            blockType = kCcBlock;
            return true;
        } else if (w == kStartWordX) {
            // When byte recieved was 0x55aa instead of otherway around, the
            // code syncs the byte
            llvm::outs() << "Pixy: reorder\n";
            GetByte();  // resync
        }
        lastw = w;
    }
}

uint16_t Pixy::GetWord() {
    uint8_t buffer[2] = {0, 0};
    i2c.ReadOnly(2, buffer);

    return (buffer[1] << 8) | buffer[0];
}

uint8_t Pixy::GetByte() {
    uint8_t buffer[1] = {0};

    i2c.ReadOnly(1, buffer);
    return buffer[0];
}

uint16_t Pixy::ConvertByte(int msb, int lsb) {
    if (msb < 0){
        msb += 256;
    }
    int value = msb * 256;

    if (lsb < 0){
        // lsb should be unsigned
        value += 256;
    }
    value += lsb;
    return value;
}

uint16_t Pixy::GetBlocks(uint16_t maxBlocks) {
    // Clear data from previous reading
    blocks[0] = {0};

    // When computer has not seen 0xaa55 (starting frame)
    if (!skipStart) {
        if (!GetStart()) {
            return 0;
        }
    } else {
        skipStart = false;
    }

    // Iterate over how many signature objects there are.
    for (uint16_t blockCount = 0;
         blockCount < maxBlocks && blockCount < kMaximumArraySize;) {
        uint16_t checksum = GetWord();
        if (checksum == kStartWord) {
            // We've reached the beginning of the next frame - checking for
            // 0xaa55.
            skipStart = true;  // starts this function
            blockType = kNormalBlock;
            return blockCount;
        } else if (checksum == kStartWordCc) {
            // We've reached the beginning of the next frame - checking for
            // 0xaa56.
            skipStart = true;
            blockType = kCcBlock;
            return blockCount;
        } else if (checksum == 0) {
            return blockCount;
        }

        Block* block = blocks + blockCount;

        i2c.ReadOnly(16, bytes);

        uint16_t sum = 0;
        for (uint8_t i = 0; i < sizeof(Block) / sizeof(uint16_t); i++) {
            if (blockType == kNormalBlock && i >= 5) {
                // Skip -- if not a CC block, no need to consider angle.
                block->angle = 0;
                break;
            }
            blocks[i].width = ConvertByte(bytes[11], bytes[10]);
            blocks[i].height = ConvertByte(bytes[13], bytes[12]);
            blocks[i].x = ConvertByte(bytes[7], bytes[6]);
            blocks[i].y = ConvertByte(bytes[9], bytes[8]);
            auto w = GetWord();
            sum += w;
            *reinterpret_cast<uint16_t*>(block + i) = w;
        }

        if (checksum == sum) {
            blockCount++;
        } else {
            llvm::outs() << "Pixy: cs error\n";
        }

        uint16_t w = GetWord();
        if (w == kStartWord) {
            // When this is start of the frame
            blockType = kNormalBlock;
        } else if (w == kStartWordCc) {
            blockType = kCcBlock;
        } else {
            return blockCount;
        }
    }
}

const Pixy::Block& Pixy::GetBlock(int blockNumber) const {
    return blocks[blockNumber];
}
