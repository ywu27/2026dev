#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include <vector>

class UIWidget {
  private:
    nt::GenericEntry *entry; //pointer to widget's entry
    std::string name; //widget's name
    std::string tab; //widget's tab

  public:
    /**
     * @param entry pointer to widget's entry
     * @param tab Widget's tab
     * @param name Widget's name
     */
    UIWidget::UIWidget(nt::GenericEntry *entry, std::string tab, std::string name) {
        this->name = name;
        this->tab = tab;
        this->entry = entry;
    }

    /**
     * Gets a UIWidget's entry
     * @return pointer to the widget's entry
     */
    nt::GenericEntry *UIWidget::GetEntry() {
        return this->entry;
    }

    /**
     * Gets a UIWidget's name
     * @return widget's name
     */
    std::string UIWidget::GetName() {
        return this->name;
    }

    /**
     * Gets a UIWidget's tab
     * @return widget's tab
     */
    std::string UIWidget::GetTab() {
        return this->tab;
    }
};