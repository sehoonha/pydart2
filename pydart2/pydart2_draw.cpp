#include "pydart2_draw.h"

void drawWorld(
    dart::gui::RenderInterface* ri,
    dart::simulation::WorldPtr world) {
    drawSkeletons(ri, world);

    for (auto i = 0u; i < world->getNumSimpleFrames(); ++i)
        drawShapeFrame(ri, world->getSimpleFrame(i).get());
}

void drawSkeletons(
    dart::gui::RenderInterface* ri,
    dart::simulation::WorldPtr world) {
    for (auto i = 0u; i < world->getNumSkeletons(); ++i)
        drawSkeleton(ri, world->getSkeleton(i).get());
}

void drawSkeleton(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Skeleton* skeleton,
    const Eigen::Vector4d& color,
    bool useDefaultColor) {
    if (!skeleton)
        return;

    for (auto i = 0u; i < skeleton->getNumTrees(); ++i)
        drawBodyNode(ri, skeleton->getRootBodyNode(i), color, useDefaultColor, true);
}

void drawEntity(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Entity* entity,
    const Eigen::Vector4d& color,
    bool useDefaultColor) {
    if (!entity)
        return;

    const auto& bodyNode = dynamic_cast<const dart::dynamics::BodyNode*>(entity);
    if (bodyNode)
    {
        drawBodyNode(ri, bodyNode, color, useDefaultColor, true);
        return;
    }

    const auto& shapeFrame = dynamic_cast<const dart::dynamics::ShapeFrame*>(entity);
    if (shapeFrame)
    {
        drawShapeFrame(ri, shapeFrame, color, useDefaultColor);
        return;
    }
}

void drawBodyNode(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::BodyNode* bodyNode,
    const Eigen::Vector4d& color,
    bool useDefaultColor,
    bool recursive) {
    if (!bodyNode)
        return;

    if (!ri)
        return;

    ri->pushMatrix();

    // Use the relative transform of this Frame. We assume that we are being
    // called from the parent Frame's renderer.
    // TODO(MXG): This can cause trouble if the draw function is originally called
    // on an Entity or Frame which is not a child of the World Frame
    ri->transform(bodyNode->getRelativeTransform());

    // _ri->pushName(???); TODO(MXG): What should we do about this for Frames?
    auto shapeNodes = bodyNode->getShapeNodesWith<dart::dynamics::VisualAspect>();
    for (const auto& shapeNode : shapeNodes)
        drawShapeFrame(ri, shapeNode, color, useDefaultColor);
    // _ri.popName();

    // if (mShowPointMasses)
    // {
    //     const auto& softBodyNode
    //         = dynamic_cast<const dart::dynamics::SoftBodyNode*>(bodyNode);
    //     if (softBodyNode)
    //         drawPointMasses(ri, softBodyNode->getPointMasses(), color);
    // }

    // if (mShowMarkers)
    // {
    //     for (auto i = 0u; i < bodyNode->getNumMarkers(); ++i)
    //         drawMarker(ri, bodyNode->getMarker(i));
    // }

    // render the subtree
    if (recursive)
    {
        for (const auto& entity : bodyNode->getChildEntities())
            drawEntity(ri, entity, color, useDefaultColor);
    }

    ri->popMatrix();
}

void drawShapeFrame(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::ShapeFrame* shapeFrame,
    const Eigen::Vector4d& color,
    bool useDefaultColor) {
    if (!shapeFrame)
        return;

    if (!ri)
        return;

    const auto& visualAspect = shapeFrame->getVisualAspect();

    if (!visualAspect || visualAspect->isHidden())
        return;

    ri->pushMatrix();
    ri->transform(shapeFrame->getRelativeTransform());

    if (useDefaultColor)
        drawShape(ri, shapeFrame->getShape().get(), visualAspect->getRGBA());
    else
        drawShape(ri, shapeFrame->getShape().get(), color);

    ri->popMatrix();
}

void drawShape(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Shape* shape,
    const Eigen::Vector4d& color) {
        if (!shape)
            return;

        if (!ri)
            return;

        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);

        ri->setPenColor(color);

        using dart::dynamics::Shape;
        using dart::dynamics::BoxShape;
        using dart::dynamics::EllipsoidShape;
        using dart::dynamics::CylinderShape;
        using dart::dynamics::PlaneShape;
        using dart::dynamics::MeshShape;
        using dart::dynamics::SoftMeshShape;
        using dart::dynamics::LineSegmentShape;

        switch (shape->getShapeType())
        {
        case Shape::BOX:
        {
            const auto& box = static_cast<const BoxShape*>(shape);
            ri->drawCube(box->getSize());

            break;
        }
        case Shape::ELLIPSOID:
        {
            const auto& ellipsoid = static_cast<const EllipsoidShape*>(shape);
            ri->drawEllipsoid(ellipsoid->getSize());

            break;
        }
        case Shape::CYLINDER:
        {
            const auto& cylinder = static_cast<const CylinderShape*>(shape);
            ri->drawCylinder(cylinder->getRadius(), cylinder->getHeight());

            break;
        }
        case Shape::MESH:
        {
            const auto& mesh = static_cast<const MeshShape*>(shape);

            glDisable(GL_COLOR_MATERIAL); // Use mesh colors to draw

            if (mesh->getDisplayList())
                ri->drawList(mesh->getDisplayList());
            else
                ri->drawMesh(mesh->getScale(), mesh->getMesh());

            break;
        }
        case Shape::SOFT_MESH:
        {
            const auto& softMesh = static_cast<const SoftMeshShape*>(shape);
            ri->drawSoftMesh(softMesh->getAssimpMesh());

            break;
        }
        case Shape::LINE_SEGMENT:
        {
            const auto& lineSegmentShape
                = static_cast<const LineSegmentShape*>(shape);
            ri->drawLineSegments(lineSegmentShape->getVertices(),
                                 lineSegmentShape->getConnections());

            break;
        }
        default:
        {
            dterr << "[SimWindow::drawShape] Attempting to draw unsupported shape "
                  << "type '" << shape->getShapeType() << "'.\n";
            break;
        }
        }

        glDisable(GL_COLOR_MATERIAL);

                                   }

void drawPointMasses(
    dart::gui::RenderInterface* ri,
    const std::vector<dart::dynamics::PointMass*> pointMasses,
    const Eigen::Vector4d& color,
    bool useDefaultColor) {
    if (!ri)
        return;

    for (const auto& pointMass : pointMasses)
    {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

        // render point at the current position
        ri->pushMatrix();
        T.translation() = pointMass->getLocalPosition();
        ri->transform(T);
        if (useDefaultColor)
            ri->setPenColor(Eigen::Vector4d(0.8, 0.3, 0.3, 1.0));
        else
            ri->setPenColor(color);
        ri->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
        ri->popMatrix();

        // render point at the resting position
        ri->pushMatrix();
        T.translation() = pointMass->getRestingPosition();
        ri->transform(T);
        if (useDefaultColor)
            ri->setPenColor(Eigen::Vector4d(0.8, 0.3, 0.3, 1.0));
        else
            ri->setPenColor(color);
        ri->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
        ri->popMatrix();
    }
}

void drawMarker(
    dart::gui::RenderInterface* ri,
    const dart::dynamics::Marker* marker,
    const Eigen::Vector4d& color,
    bool useDefaultColor) {
    if (!marker)
        return;

    if (!ri)
        return;

    ri->pushName(marker->getID());

    if (marker->getConstraintType() == dart::dynamics::Marker::HARD)
    {
        ri->setPenColor(Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
    }
    else if (marker->getConstraintType() == dart::dynamics::Marker::SOFT)
    {
        ri->setPenColor(Eigen::Vector4d(0.0, 1.0, 0.0, 1.0));
    }
    else
    {
        if (useDefaultColor)
            ri->setPenColor(marker->getColor());
        else
            ri->setPenColor(color);
    }

    ri->pushMatrix();
    // ri->translate(marker->getLocalPosition());
    ri->translate(marker->getWorldPosition());
    ri->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
    ri->popMatrix();

    ri->popName();
}

void drawContact(
    dart::gui::RenderInterface* ri,
    const Eigen::Vector6d& state,
    double size,
    double scale) {
    glEnable(GL_COLOR_MATERIAL);

    ri->setPenColor(Eigen::Vector4d(1.0, 0.0, 0.0, 1.0));
    Eigen::Vector3d p = state.head<3>();
    Eigen::Vector3d f = state.tail<3>();
    Eigen::Vector3d p2 = p + scale * f;
    std::vector<Eigen::Vector3d> verts;
    verts.push_back(p);
    verts.push_back(p2);
    Eigen::aligned_vector<Eigen::Vector2i> conn;
    conn.push_back(Eigen::Vector2i(0, 1));

    
    ri->pushMatrix();
    ri->translate(p);
    ri->drawEllipsoid(Eigen::Vector3d::Constant(0.01));
    ri->popMatrix();
    ri->drawLineSegments(verts, conn);
}
