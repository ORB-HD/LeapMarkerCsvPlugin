#ifndef LEAPMARKERCSVPLUGIN_H_INCLUDED
#define LEAPMARKERCSVPLUGIN_H_INCLUDED

#include <QWidget>
#include <QString>
#include <QStringList>

#include "toolkit_interfaces.h"
#include "toolkit_errors.h"
#include "util.h"

#include "plugins/MotionMarkerPlugin/MotionMarkerExtension.h"

class LeapMarkerCsvPlugin : public QObject, public OptionalInterface {
	Q_OBJECT
	Q_INTERFACES(OptionalInterface)
	Q_PLUGIN_METADATA(IID OptionalInterface_iid FILE "metadata.json")
	public:
		LeapMarkerCsvPlugin();
		virtual ~LeapMarkerCsvPlugin();

		void init(ToolkitApp* app);
		MotionMarkerExtension* loadLeapMarkerCsvFile(QString path);
	private:
		ToolkitApp* parentApp;
		QAction* load_file_trigger;
		void loadLeapMarkerCsvSettings();
		void addMarkerFrameToExt(MotionMarkerExtension* ext, int marker_count, std::vector<QString>& names, std::vector<float>& row_values, bool);
		std::map<RBDLModelWrapper*, QString> model_file_map;

		char csv_seperator;
		float marker_size;
		QColor marker_color;

		QStringList x_suffix;
		QStringList y_suffix;
		QStringList z_suffix;
		QStringList all_suffix;

	public Q_SLOTS:
		void action_load_data();
		void reload(RBDLModelWrapper* model);

};

#endif 
