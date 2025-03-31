#pragma once

#include <memory>
#include <string>
#include <vector>

#include "CheckersConsts.hpp"

class TurtlePiece
{
public:
    TurtlePiece(
        const std::string &name,
        TurtlePieceColor color);

    std::string &getName();

    TurtlePieceColor getColor();

    bool getIsMovable();
    void setIsMovable(bool isMovable);
    void toggleIsMovable();

    bool getIsHighlighted();
    void setIsHighlighted(bool isHighlighted);
    void toggleIsHighlighted();

    bool getIsSelected();
    void setIsSelected(bool isSelected);
    void toggleIsSelected();

    bool getIsKinged();
    void setIsKinged(bool isKinged);
    void toggleIsKinged();

    bool getIsDead();
    void setIsDead(bool isDead);
    void toggleIsDead();

protected:
    std::string m_name;

    TurtlePieceColor m_color;

    bool m_isMovable;
    bool m_isHighlighted;
    bool m_isSelected;
    bool m_isKinged;
    bool m_isDead;
};

typedef std::shared_ptr<TurtlePiece> TurtlePiecePtr;