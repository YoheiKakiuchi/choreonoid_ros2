#ifndef CNOID_ROS_PLUGIN_SCENE_PUBLISHER_ITEM_H
#define CNOID_ROS_PLUGIN_SCENE_PUBLISHER_ITEM_H

#include "exportdecl.h"
#include <cnoid/Item>

namespace cnoid {

class CNOID_EXPORT ScenePublisherItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    ScenePublisherItem();
    ScenePublisherItem(const ScenePublisherItem& org);
    virtual ~ScenePublisherItem();

protected:
    virtual Item* doDuplicate() const override;
    //virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<ScenePublisherItem> ScenePublisherPtr;

}  // namespace cnoid

#endif
