void addBlockToBelt(std::stack<Block>* blocks, Block blockToAdd) {
  blocks->push(blockToAdd);
  return;
}

Block getNextBlock(std::stack<Block>* blocks) {
  Block topBlock = blocks->top();
  blocks->pop();
  return topBlock;
}

Block createBlock(RGB rgb) {
  Block newBlock;
  BlockColor color = predictColor(rgb);

  switch (color) {
    case BlockColor::Red:
      DEBUG_PRINTLN("Red block detected");
      newBlock.color = BlockColor::Red;
      break;
    case BlockColor::Yellow:
      DEBUG_PRINTLN("Yellow block detected");
      newBlock.color = BlockColor::Yellow;
      break;
    case BlockColor::Blue:
      DEBUG_PRINTLN("Blue block detected");
      newBlock.color = BlockColor::Blue;
      break;
    case BlockColor::None:
    default:
      DEBUG_PRINTLN("Uncertain about the color");
      DEBUG_PRINT("RGB: (");
      DEBUG_PRINT(rgb.r);
      DEBUG_PRINT(", ");
      DEBUG_PRINT(rgb.g);
      DEBUG_PRINT(", ");
      DEBUG_PRINT(rgb.b);
      DEBUG_PRINTLN(")");
      DEBUG_PRINTLN("Setting color to None to avoid crashing");
      newBlock.color = BlockColor::None;
  }

  return newBlock;
}


void addBlockToStackFromRGB(std::stack<Block>* blocks, RGB rgb) {
  Block newBlock = createBlock(rgb);

  addBlockToBelt(blocks, newBlock);
}