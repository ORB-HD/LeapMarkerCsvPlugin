#include "LeapMarkerCsvPlugin.h"
#include <iostream>

#include <QFileDialog>
#include <QMessageBox>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <rbdl_wrapper.h>

#include <parser.hpp>

using namespace RigidBodyDynamics::Math;

LeapMarkerCsvPlugin::LeapMarkerCsvPlugin() {
	parentApp = NULL;

	x_suffix << "x" << "_x";
	y_suffix << "y" << "_y";
	z_suffix << "z" << "_z";
	all_suffix << x_suffix << y_suffix << z_suffix;
}

LeapMarkerCsvPlugin::~LeapMarkerCsvPlugin() {
}

void LeapMarkerCsvPlugin::init(ToolkitApp* app) {
	//save reference to parent ToolkitApp 
	parentApp = app;

	load_file_trigger = new QAction("Load LeapMarkerCsv");
	parentApp->addFileAction(load_file_trigger);

	connect(load_file_trigger, SIGNAL(triggered(bool)), this, SLOT(action_load_data()));

	QCommandLineOption leapmarkercsv_option( QStringList() << "leapmarkercsv",
	                                 "Load LeapMarkerCsv files <file>", 
	                                 "file"
	                               );
	parentApp->addCmdOption(leapmarkercsv_option, [this](QCommandLineParser& parser){
		auto data_list = parser.values("leapmarkercsv");
		for (int i=0; i<data_list.size(); i++) {
			if (i < parentApp->getLoadedModels()->size() ) {
				auto file = data_list[i];
				MotionMarkerExtension* ext = nullptr;
				try {
					ext = loadLeapMarkerCsvFile(file);
				} catch (RigidBodyDynamics::Errors::RBDLError& e){
					ToolkitApp::showExceptionDialog(e);
					if (ext != nullptr)
						delete ext;
					continue;
				}
				if (ext != nullptr) {
					RBDLModelWrapper* rbdl_model = parentApp->getLoadedModels()->at(i);
					rbdl_model->addExtension(ext);
					model_file_map[rbdl_model] = file;
					parentApp->getToolkitTimeline()->setMaxTime(ext->getMaxTime());
				}
			} else {
				std::cout << QString("LeapMarkerCsv file %1 can not be mapped to a model ... Ignoring!").arg(data_list[i]).toStdString() << std::endl;
			}
		}
		
	});

	loadLeapMarkerCsvSettings();

	std::cout << "LeapMarkerCsvPlugin loaded" << std::endl;

	connect(parentApp, &ToolkitApp::reloaded_model, this, &LeapMarkerCsvPlugin::reload);
}


void LeapMarkerCsvPlugin::loadLeapMarkerCsvSettings() {
	parentApp->toolkit_settings.beginGroup("FileReaderOptions");

	//seperator setting
	QVariant val = parentApp->toolkit_settings.value("csv.seperator");
	if (val.isNull()) {
		csv_seperator = ',';
		parentApp->toolkit_settings.setValue("csv.seperator", csv_seperator);
	} else {
		//read as int because it is saved as one, otherwise it would not load correctly
		//watch out for overflow -> will silently fail because char is uint8
		csv_seperator = val.toInt();
	}

	parentApp->toolkit_settings.endGroup();
	parentApp->toolkit_settings.beginGroup("MarkerOptions");

	//marker color
	val = parentApp->toolkit_settings.value("marker.color");
	if (val.isNull()) {
		marker_color = QColor::fromRgbF(0., 0., 1., 1.);
		parentApp->toolkit_settings.setValue("marker.color", marker_color.rgba());
	} else {
		marker_color = QColor::fromRgba(val.toUInt());
	}
	parentApp->toolkit_settings.setType("marker.color", marker_color);

	//marker size
	val = parentApp->toolkit_settings.value("marker.size");
	if (val.isNull()) {
		marker_size = 0.01;
		parentApp->toolkit_settings.setValue("marker.size", marker_size);
	} else {
		marker_size = val.toFloat();
	}
	parentApp->toolkit_settings.setType("marker.size", marker_size);

	parentApp->toolkit_settings.endGroup();
}

void LeapMarkerCsvPlugin::action_load_data() {
	if (parentApp != NULL) {
		QFileDialog file_dialog (parentApp, "Select LeapMarkerCsv File");

		//file_dialog.setNameFilter(tr("LeapMarkerCsv File (*.txt)"));
		file_dialog.setFileMode(QFileDialog::ExistingFile);

		MotionMarkerExtension* ext;
		if (file_dialog.exec()) {
			QString filepath = file_dialog.selectedFiles().at(0);
			try {
				ext = loadLeapMarkerCsvFile(filepath);
			} catch (RigidBodyDynamics::Errors::RBDLError& e){
				ToolkitApp::showExceptionDialog(e);
				if (ext != nullptr) delete ext;
			}
			if (ext != nullptr) {
				if (parentApp->getLoadedModels()->size() != 0) {
					RBDLModelWrapper* rbdl_model = nullptr;

					if (parentApp->getLoadedModels()->size() == 1) {
						rbdl_model = parentApp->getLoadedModels()->at(0);
					} else {
						rbdl_model = parentApp->selectModel(nullptr);
					}

					if (rbdl_model != nullptr) {
						rbdl_model->addExtension(ext);
						model_file_map[rbdl_model] = file_dialog.selectedFiles().at(0);
						parentApp->getToolkitTimeline()->setMaxTime(ext->getMaxTime());
					} else {
						delete ext;
					}
				}
			}
		}	
	} else {
		//should never happen
		throw RigidBodyDynamics::Errors::RBDLError("LeapMarkerCsvPlugin was not initialized correctly!");
	}
}

void LeapMarkerCsvPlugin::addMarkerFrameToExt(MotionMarkerExtension* ext, int marker_count ,std::vector<QString>& names, std::vector<float>& row_values, bool set_names) {
	float time = row_values[0];
	Matrix3fd pos;
	pos.resize(3, marker_count);

	QStringList marker_names;

	for (int i=1; i < names.size(); i++) {
		int vector_index = -1;
		int suffix_len=0;
		for (int j=0; j < x_suffix.size(); j++) {
			if (names[i].endsWith(x_suffix[j], Qt::CaseInsensitive)) {
				vector_index = 0;
				suffix_len = x_suffix[j].size();
			} else if (names[i].endsWith(y_suffix[j], Qt::CaseInsensitive)) {
				vector_index = 1;
				suffix_len = y_suffix[j].size();
			} else if (names[i].endsWith(z_suffix[j], Qt::CaseInsensitive)) {
				vector_index = 2;
				suffix_len = z_suffix[j].size();
			}
		}

		if (vector_index == -1) {
			throw RigidBodyDynamics::Errors::RBDLFileParseError(QString("Could not determine if the value of %1 corresponds to the x,y or z coordinate. Please use format <markername>x or <markername>_x").arg(names[i]).toStdString());
		}
		QString marker_name = names[i].left(names[i].size()-suffix_len);

		if (!marker_names.contains(marker_name)) {
			marker_names << marker_name;
		}
		int matrix_index = marker_names.indexOf(marker_name);

		// set value und marker frame matrix
		pos(vector_index, matrix_index) = row_values[i] * 1.0e-3;
	}
	ext->addMocapMarkerFrame(time, pos);

	if (set_names) {
		for (int i=0; i < marker_names.size(); i++) {
			std::string marker_name = marker_names[i].toStdString();
			ext->setMarkerLabel(i, marker_name);
		}
	}
}

MotionMarkerExtension* LeapMarkerCsvPlugin::loadLeapMarkerCsvFile(QString path) {
	MotionMarkerExtension* extension = nullptr;

	std::setlocale(LC_NUMERIC, "en_US.UTF-8");

	std::vector<QString> names;
	std::vector<float> row_values;

	std::ifstream file(path.toStdString().c_str(), std::ios_base::in);
	aria::csv::CsvParser parser(file);
	parser.delimiter(csv_seperator);

	bool header_check = false;
	int marker_count;
	bool set_names = true;

	for (auto& row : parser) {
		bool ok = true;
		if (!header_check) {
			header_check = true;

			QString first_entry = QString::fromStdString(row[0]);
			first_entry.toFloat(&ok);
			if (!ok) {
				//found header
				for (auto& field : row) {
					names.push_back(QString::fromStdString(field));
				}
			} else {
				// no header was found, cannot assign values to Marker names, so we cannot
				// load this file correctly
				throw RigidBodyDynamics::Errors::RBDLFileParseError("Leap csv does not contain a header, so we cannot load it since we cannot know what the values reference which markers!");
			}

			// first col is always the time: check that there amount of other columns is
			// a multiple of 3 since we are reading 3D Vectors for ever motion marker
			if ( (names.size()-1) % 3 != 0 ) {
				throw RigidBodyDynamics::Errors::RBDLFileParseError("Marker columns in header are not a multiple of 3, but we need 3D Values for every marker!");
			}

			// now create extension, because we know how many markers we need to load
			marker_count = (names.size()-1)/3;
			extension = new MotionMarkerExtension(marker_count, marker_color, marker_size);
			continue;
		}

		for (auto& field : row) {
			row_values.push_back(QString::fromStdString(field).toFloat(&ok));
		}

		addMarkerFrameToExt(extension, marker_count, names, row_values, set_names);
		set_names = false;

		row_values.clear();
	}


	return extension;
}

void LeapMarkerCsvPlugin::reload(RBDLModelWrapper *model) {
	for (auto it = model_file_map.begin(); it != model_file_map.end(); it++) {
		if ( it->first == model ) {
			auto ext = loadLeapMarkerCsvFile(it->second);
			model->addExtension(ext);
		}
	}
}
