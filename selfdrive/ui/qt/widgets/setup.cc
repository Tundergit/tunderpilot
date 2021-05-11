#include "setup.h"

#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QPushButton>
#include <QStackedWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QrCode.hpp>

#include "selfdrive/common/params.h"
#include "selfdrive/ui/qt/request_repeater.h"

using qrcodegen::QrCode;

PairingQRWidget::PairingQRWidget(QWidget* parent) : QWidget(parent) {
  qrCode = new QLabel;
  qrCode->setScaledContents(true);
  QVBoxLayout* v = new QVBoxLayout;
  v->addWidget(qrCode, 0, Qt::AlignCenter);
  setLayout(v);

  QTimer* timer = new QTimer(this);
  timer->start(30 * 1000);
  connect(timer, &QTimer::timeout, this, &PairingQRWidget::refresh);
}

void PairingQRWidget::showEvent(QShowEvent *event){
  refresh();
}

void PairingQRWidget::refresh(){
  Params params;
  QString IMEI = QString::fromStdString(params.get("IMEI"));
  QString serial = QString::fromStdString(params.get("HardwareSerial"));

  if (std::min(IMEI.length(), serial.length()) <= 5) {
    qrCode->setText("Error getting serial: contact support");
    qrCode->setWordWrap(true);
    qrCode->setStyleSheet(R"(font-size: 60px;)");
    return;
  }
  QString pairToken = CommaApi::create_jwt({{"pair", true}});

  QString qrString = IMEI + "--" + serial + "--" + pairToken;
  this->updateQrCode(qrString);
}

void PairingQRWidget::updateQrCode(QString text) {
  QrCode qr = QrCode::encodeText(text.toUtf8().data(), QrCode::Ecc::LOW);
  qint32 sz = qr.getSize();
  // make the image larger so we can have a white border
  QImage im(sz + 2, sz + 2, QImage::Format_RGB32);
  QRgb black = qRgb(0, 0, 0);
  QRgb white = qRgb(255, 255, 255);

  for (int y = 0; y < sz + 2; y++) {
    for (int x = 0; x < sz + 2; x++) {
      im.setPixel(x, y, white);
    }
  }
  for (int y = 0; y < sz; y++) {
    for (int x = 0; x < sz; x++) {
      im.setPixel(x + 1, y + 1, qr.getModule(x, y) ? black : white);
    }
  }
  // Integer division to prevent anti-aliasing
  int approx500 = (500 / (sz + 2)) * (sz + 2);
  qrCode->setPixmap(QPixmap::fromImage(im.scaled(approx500, approx500, Qt::KeepAspectRatio, Qt::FastTransformation), Qt::MonoOnly));
  qrCode->setFixedSize(approx500, approx500);
}

PrimeUserWidget::PrimeUserWidget(QWidget* parent) : QWidget(parent) {
  mainLayout = new QVBoxLayout;
  mainLayout->setMargin(30);

  QLabel* commaPrime = new QLabel("COMMA PRIME");
  mainLayout->addWidget(commaPrime, 0, Qt::AlignTop);

  username = new QLabel();
  username->setStyleSheet("font-size: 55px;"); // TODO: fit width
  mainLayout->addWidget(username, 0, Qt::AlignTop);

  mainLayout->addSpacing(100);

  QLabel* commaPoints = new QLabel("COMMA POINTS");
  commaPoints->setStyleSheet(R"(
    color: #b8b8b8;
  )");
  mainLayout->addWidget(commaPoints, 0, Qt::AlignTop);

  points = new QLabel();
  mainLayout->addWidget(points, 0, Qt::AlignTop);

  setLayout(mainLayout);
  setStyleSheet(R"(
    QLabel {
      font-size: 70px;
      font-weight: 500;
    }
  )");

  // set up API requests
  QString dongleId = QString::fromStdString(Params().get("DongleId"));
  if (!dongleId.length()) {
    return;
  }

  QString url = "https://api.commadotai.com/v1/devices/" + dongleId + "/owner";
  RequestRepeater *repeater = new RequestRepeater(this, url, "ApiCache_Owner", 6);
  QObject::connect(repeater, &RequestRepeater::receivedResponse, this, &PrimeUserWidget::replyFinished);
}

void PrimeUserWidget::replyFinished(const QString &response) {
  QJsonDocument doc = QJsonDocument::fromJson(response.toUtf8());
  if (doc.isNull()) {
    qDebug() << "JSON Parse failed on getting username and points";
    return;
  }

  QJsonObject json = doc.object();
  QString points_str = QString::number(json["points"].toInt());
  QString username_str = json["username"].toString();
  if (username_str.length()) {
    username_str = "@" + username_str;
  }

  username->setText(username_str);
  points->setText(points_str);
}

PrimeAdWidget::PrimeAdWidget(QWidget* parent) : QWidget(parent) {
  QVBoxLayout* vlayout = new QVBoxLayout;
  vlayout->setMargin(30);
  vlayout->setSpacing(15);

  vlayout->addWidget(new QLabel("Upgrade now"), 1, Qt::AlignTop);

  QLabel* description = new QLabel("Become a comma prime member in the comma connect app and get premium features!");
  description->setStyleSheet(R"(
    font-size: 50px;
    color: #b8b8b8;
  )");
  description->setWordWrap(true);
  vlayout->addWidget(description, 2, Qt::AlignTop);

  QVector<QString> features = {"✓ REMOTE ACCESS", "✓ 14 DAYS OF STORAGE", "✓ DEVELOPER PERKS"};
  for (auto &f: features) {
    QLabel* feature = new QLabel(f);
    feature->setStyleSheet(R"(font-size: 40px;)");
    vlayout->addWidget(feature, 0, Qt::AlignBottom);
  }

  setLayout(vlayout);
}


SetupWidget::SetupWidget(QWidget* parent) : QFrame(parent) {
  mainLayout = new QStackedWidget;

  // Unpaired, registration prompt layout

  QVBoxLayout* finishRegistationLayout = new QVBoxLayout;
  finishRegistationLayout->setMargin(30);

  QLabel* registrationDescription = new QLabel("Pair your device with the comma connect app");
  registrationDescription->setWordWrap(true);
  registrationDescription->setAlignment(Qt::AlignCenter);
  registrationDescription->setStyleSheet(R"(
    font-size: 55px;
    font-weight: 400;
  )");

  finishRegistationLayout->addWidget(registrationDescription);

  QPushButton* finishButton = new QPushButton("Finish setup");
  finishButton->setFixedHeight(200);
  finishButton->setStyleSheet(R"(
    border-radius: 30px;
    font-size: 55px;
    font-weight: 500;
    background: #585858;
  )");
  finishRegistationLayout->addWidget(finishButton);
  QObject::connect(finishButton, &QPushButton::released, this, &SetupWidget::showQrCode);

  QWidget* finishRegistration = new QWidget;
  finishRegistration->setLayout(finishRegistationLayout);
  mainLayout->addWidget(finishRegistration);

  // Pairing QR code layout

  QVBoxLayout* qrLayout = new QVBoxLayout;

  qrLayout->addSpacing(30);
  QLabel* qrLabel = new QLabel("Scan with comma connect!");
  qrLabel->setWordWrap(true);
  qrLabel->setAlignment(Qt::AlignHCenter);
  qrLabel->setStyleSheet(R"(
    font-size: 55px;
    font-weight: 400;
  )");
  qrLayout->addWidget(qrLabel, 0, Qt::AlignTop);

  qrLayout->addWidget(new PairingQRWidget, 1);

  QWidget* q = new QWidget;
  q->setLayout(qrLayout);
  mainLayout->addWidget(q);

  primeAd = new PrimeAdWidget;
  mainLayout->addWidget(primeAd);

  primeUser = new PrimeUserWidget;
  mainLayout->addWidget(primeUser);

  mainLayout->setCurrentWidget(primeAd);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addWidget(mainLayout);
  setLayout(layout);

  setStyleSheet(R"(
    SetupWidget {
      background-color: #292929;
    }
    * {
      font-size: 90px;
      font-weight: 500;
      border-radius: 40px;
    }
  )");

  // Retain size while hidden
  QSizePolicy sp_retain = sizePolicy();
  sp_retain.setRetainSizeWhenHidden(true);
  setSizePolicy(sp_retain);

  // set up API requests
  QString dongleId = QString::fromStdString(Params().get("DongleId"));
  QString url = "https://api.commadotai.com/v1.1/devices/" + dongleId + "/";
  RequestRepeater* repeater = new RequestRepeater(this, url, "ApiCache_Device", 5);

  QObject::connect(repeater, &RequestRepeater::receivedResponse, this, &SetupWidget::replyFinished);
  QObject::connect(repeater, &RequestRepeater::failedResponse, this, &SetupWidget::parseError);
  hide(); // Only show when first request comes back
}

void SetupWidget::parseError(const QString &response) {
  show();
  showQr = false;
  mainLayout->setCurrentIndex(0);
}

void SetupWidget::showQrCode(){
  showQr = true;
  mainLayout->setCurrentIndex(1);
}

void SetupWidget::replyFinished(const QString &response) {
  show();
  QJsonDocument doc = QJsonDocument::fromJson(response.toUtf8());
  if (doc.isNull()) {
    qDebug() << "JSON Parse failed on getting pairing and prime status";
    return;
  }

  QJsonObject json = doc.object();
  bool is_paired = json["is_paired"].toBool();
  bool is_prime = json["prime"].toBool();

  if (!is_paired) {
    mainLayout->setCurrentIndex(showQr);
  } else if (!is_prime) {
    showQr = false;
    mainLayout->setCurrentWidget(primeAd);
  } else {
    showQr = false;
    mainLayout->setCurrentWidget(primeUser);
  }
}
