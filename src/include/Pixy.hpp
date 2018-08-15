// Copyright (c) 2018 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <I2C.h>

class Pixy {
public:
    // Default address of Pixy Camera. You can change the address of the Pixy in
    // Pixymon under setting-> Interface
    static constexpr uint8_t kI2cDefaultAddr = 0x54;

    // Communication/misc parameters

    static constexpr int kInitialArraySize = 30;
    static constexpr int kMaximumArraySize = 130;

    // For regular color recognition
    static constexpr int kStartWord = 0xaa55;

    // For color code - angle rotation recognition
    static constexpr int kStartWordCc = 0xaa56;

    // Regular color another way around
    static constexpr int kStartWordX = 0x55aa;

    static constexpr int kMaxSignature = 7;
    static constexpr int kDefaultArgVal = 0xffff;

    // Pixy x-y position values.
    // x: 0-319 pixels, y: 0~199 pixels, (0, 0) starts at bottom left.
    static constexpr int kMinX = 0;
    static constexpr int kMaxX = 319;
    static constexpr int kMinY = 0;
    static constexpr int kMaxY = 199;

    // RC-servo values - not needed unless you want to use servo to face the
    // goal instead of moving the whole robot
    static constexpr int kRcsMinPos = 0;
    static constexpr int kRcsMaxPos = 1000;
    static constexpr int kRcCenterPos = (kRcsMaxPos - kRcsMinPos) / 2;

    struct Block {
        // Identification number for your object - you could set it in pixymon
        uint16_t signature;

        // 0-320 pixels
        uint16_t x;

        // 0-200 pixels
        uint16_t y;

        uint16_t width;
        uint16_t height;

        // Only appears when using Color Code
        uint16_t angle;

        /**
         * Prints block structure - prints pixy stat (xy coordinates, height,
         * width, etc.).
         */
        void Print() const;
    };

    enum BlockType {
        /**
         * Regular color recognition.
         */
        kNormalBlock,

        /**
         * Color-Code recognition (determines how much object is tilted).
         */
        kCcBlock
    };

    /**
     * Checks if the frame is a new frame.
     */
    bool GetStart();

    /**
     * Gets two bytes from the Pixy (the full information).
     */
    uint16_t GetWord();

    /**
     * Gets a byte from the Pixy.
     */
    uint8_t GetByte();

    uint16_t ConvertByte(int msb, int lsb);

    /**
     * Returns how many (signature) objects are detected.
     */
    uint16_t GetBlocks(uint16_t maxBlocks);

    /**
     * Returns reference to the given block.
     *
     * @param blockNumber Number of object.
     */
    const Block& GetBlock(int blockNumber) const;

private:
    frc::I2C i2c{I2C::Port::kOnboard, kI2cDefaultAddr};
    BlockType blockType = kNormalBlock;
    uint8_t bytes[280] = {0};

    /**
     * Skips to check 0xaa55, which is byte that tells Pixy a new frame has
     * started.
     */
    bool skipStart = false;

    /**
     * Unused.
     */
    uint16_t blockArraySize = 0;

    /**
     * Array that stores blockCount array.
     */
    Block blocks[100];
};
